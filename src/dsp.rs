use micromath::F32Ext;
use core::f32::consts::PI;


use cmsis_dsp_sys_pregenerated::{arm_fir_f32, arm_fir_instance_f32, arm_fir_init_f32};

//the hardware requires a state buffer sized: BLOCK_SIZE + NUM_TAPS - 1
//we process 1024 floats per loop. filter is <100taps
const BLOCK_SIZE: usize = 1024;
const MAX_TAPS: usize = 100;


//sine generator used for testing
pub fn fill_sine_buffer(buffer: &mut [f32], phase: &mut f32, phase_step: f32) {
    for f_sample in buffer.iter_mut() {
        *f_sample = phase.sin();
        *phase += phase_step;
        if *phase > 2.0*PI {
            *phase -= 2.0*PI;
        }
    }
}

//take float buffer and make it u32 for dma write
pub fn prep_for_i2s(out_buffer: &mut [u32], in_buffer: &[f32]) {
    for (f_sample, out_chunk) in in_buffer.iter().zip(out_buffer.chunks_mut(2)) { //rust is amazing, it has a chunks mut thing, autocomplete suggested it
        let w_sample = (*f_sample * 32000.0) as i16;
        let val_u32 = (w_sample as u16) as u32;
        let word = val_u32 << 16;
        out_chunk[0] = word;
        out_chunk[1] = word;
    }
}

pub fn prep_for_processing(in_buffer: &[u32], out_buffer: &mut [f32]) {
    for (in_chunk, f_sample) in in_buffer.chunks(2).zip(out_buffer.iter_mut()) {
        let signed_word = in_chunk[0] as i32;
        *f_sample = (signed_word as f32) / 2147483648.0;
    }
}

pub static OD_AA_FIR_COEFFS: [f32; 35] = [
    0.0032304017,
    0.0060759826,
    0.0014666575, 
    -0.0112035644,
    -0.017688207,
    -0.0049748515,
    0.014155756,
    0.010418344,
    -0.017653248,
    -0.028644158,
    0.006274823,
    0.045331154,
    0.017816578,
    -0.062994796,
    -0.075201644,
    0.07472586,
    0.307133,
    0.42060977,   
    0.307133,
    0.07472586,
    -0.075201644,
    -0.062994796,
    0.017816578,
    0.045331154,
    0.006274823,
    -0.028644158,
    -0.017653248,
    0.010418344,
    0.014155756,
    -0.0049748515,
    -0.017688207,
    -0.0112035644,
    0.0014666575,
    0.0060759826,
    0.0032304017   // h(0), its symmetrical anyways
];


//udo zolzer overdive
pub struct overdrive_params {
    pub pre_gain: f32,
    pub post_gain: f32,
}

impl Default for overdrive_params {
    fn default() -> Self {
        Self {
            pre_gain: 1.0,
            post_gain: 1.0,
        }
    }
}

pub fn overdrive(buffer: &mut[f32], overdrive_params: &overdrive_params) {
    for sample in buffer.iter_mut() {
        let clipIn = *sample * overdrive_params.pre_gain;
        let absClipIn = clipIn.abs();
        let signClipIn = if clipIn < 0.0 { -1.0 } else { 1.0 };

        let threshold = 1.0 / 3.0;

        if absClipIn < threshold {
            *sample = 2.0 * clipIn;
        }
        else if (absClipIn >= threshold) && (absClipIn < 2.0 * threshold) {
            *sample = signClipIn * (3.0 - (2.0 - 3.0 * absClipIn) * (2.0 - 3.0 * absClipIn)) / 3.0;
        }
        else {
            *sample = signClipIn;
        }

        if *sample > 1.0 {
            *sample = 1.0;
        }
        else if *sample < -1.0 {
            *sample = -1.0;
        }

        *sample *= overdrive_params.post_gain;
    }
}


//35 tap fir filter for anti aliasing, passband 0-9k, stopband 10k-24k(i think, it may actually be 12k-24k but i forgot)
pub struct AntiAliasFir{
    coeffs: [f32; 35],
    state: [f32; 1058]
}

impl AntiAliasFir {
    pub fn new(coeffs: [f32; 35]) -> Self {
        Self {
            coeffs,
            state: [0.0; 1058]
        }
    }

    pub fn process(&mut self, input: &[f32], output: &mut[f32]) {
        self.state[34..1058].copy_from_slice(input); //last buffer is up to 34, 34+1024(new buffer) = 1058;, we copy the new buffer into the 1024 bytes
        for (i, out_samp) in output.iter_mut().enumerate() {
            let mut sum = 0.0;
            let window = &self.state[i .. i + 35]; //window we slide around
            
            for (x, c) in window.iter().zip(self.coeffs.iter()) {
                sum += x * c;
            }
            *out_samp = sum;
        }
        self.state.copy_within(1024..1058, 0);
    }
}

pub struct sampler {
    raw_bytes: &'static [u8],
    cursor: usize
}

impl sampler {
    pub fn new(raw_bytes: &'static [u8]) -> Self {
        Self {
            raw_bytes,
            cursor: 0,
        }
    }

    pub fn fill_buffer(&mut self,buffer: &mut[f32]) {
        for sample in buffer.iter_mut() {
            if self.cursor + 1 >= self.raw_bytes.len() {
                self.cursor = 0;
            }

            let byte1 = self.raw_bytes[self.cursor];
            let byte2 = self.raw_bytes[self.cursor+1];
            let i16_sample = i16::from_le_bytes([byte1, byte2]);
            *sample = (i16_sample as f32) / 32768.0;
            self.cursor +=2;
        };
    }
}


//cmsis dsp reduces my loop processing time from like 1400us to 1100us
//granted, this is an old version, but im doubting that matters, i dont know if i should keep this

//update: i found this way to make it faster(we dont gotta clean up the c pointers every time but its a bit more unsafe, gains like 100us more)

pub struct CmsisFir {
    instance: arm_fir_instance_f32,
    state: [f32; 1024 + 64], 
}

impl CmsisFir {
    pub fn new() -> Self {
        Self {
            instance: unsafe { core::mem::zeroed() },
            state: [0.0; 1088],
        }
    }

    pub fn init(&mut self) {
        unsafe {
            arm_fir_init_f32(
                &mut self.instance,
                OD_AA_FIR_COEFFS.len() as u16,
                OD_AA_FIR_COEFFS.as_ptr(),
                self.state.as_mut_ptr(),
                1024,
            );
        }
    }

    pub fn process(&mut self, input: &[f32], output: &mut [f32]) {
        unsafe {
            //no more pointer updates
            arm_fir_f32(&self.instance, input.as_ptr(), output.as_mut_ptr(), 1024);
        }
    }
}

pub struct IIRHpf1 {
    x: [f32; 2],
    y: [f32; 2],
    wc_t: f32,
    fs: f32
}

impl IIRHpf1 {
    pub fn new(fs: f32, fc: f32) -> Self {
        Self {
            fs,
            x: [0.0, 0.0],
            y: [0.0, 0.0],
            wc_t: 2.0*PI*fc/fs,
        }
    }

    pub fn set_cutoff(&mut self,  fc:f32) {
        self.wc_t = 2.0*PI*fc/self.fs;
    }

    pub fn process(&mut self, buffer: &mut [f32]) {
        for sample in buffer.iter_mut() {
            self.x[1] = self.x[0];
            self.x[0] = *sample;
            let y_new = (2.0*(self.x[0]-self.x[1])+(2.0 - self.wc_t)*self.y[0])/(2.0+self.wc_t);
            self.y[1] = self.y[0];
            self.y[0] = y_new;
            *sample = y_new;
        }
    }
}


pub struct IIRLpf1 {
    x: [f32; 3],
    y: [f32; 3],
    wc_t: f32,
    fs: f32,
    damp: f32
}

impl IIRLpf1 {
    pub fn new(fs: f32, fc: f32, damp: f32) -> Self {
        Self {
            fs,
            damp,
            x: [0.0; 3],
            y: [0.0; 3],
            wc_t: 2.0*PI*fc/fs,
        }
    }

    pub fn set_cutoff_damp(&mut self,  fc:f32, damp: f32) {
        self.wc_t = 2.0*PI*fc/self.fs;
        self.damp = damp;
    }

    pub fn process(&mut self, buffer: &mut [f32]) {
        let wc_t2 = self.wc_t * self.wc_t;
        let inv_denom = 1.0 / (4.0 + 4.0 * self.damp * self.wc_t + wc_t2);
        let a1 = -2.0 * (wc_t2 - 4.0);                    //coeff on y[n-1]
        let a2 = -(4.0 - 4.0 * self.damp * self.wc_t + wc_t2);      //coeff on y[n-2]
        for sample in buffer.iter_mut() {
            self.x[2] = self.x[1];
            self.x[1] = self.x[0];
            self.x[0] = *sample;
            let y_new = wc_t2 * (self.x[0] + 2.0 * self.x[1] + self.x[2])
                + a1 * self.y[0]
                + a2 * self.y[1];
            let y_new = y_new * inv_denom;
            self.y[2] = self.y[1];
            self.y[1] = self.y[0];
            self.y[0] = y_new;

            *sample = y_new;
        }
    }
}

//noise gate: moving-RMS detector + binary gate + asymmetric IIR smoothing with hold debounce.
//ported from phil's lab IFX_NoiseGate.
const NG_RMS_WINDOW: usize = 480; //10 ms at 48 kHz

pub struct MovingRms {
    buf: [f32; NG_RMS_WINDOW],
    head: usize,
    sum: f32,
}

impl MovingRms {
    pub fn new() -> Self {
        Self { buf: [0.0; NG_RMS_WINDOW], head: 0, sum: 0.0 }
    }

    //returns mean-square (not sqrt'd) so caller can compare against threshold^2 and skip a sqrt.
    pub fn update(&mut self, x: f32) -> f32 {
        let sq = x * x;
        self.sum -= self.buf[self.head];
        self.sum += sq;
        self.buf[self.head] = sq;
        self.head += 1;
        if self.head >= NG_RMS_WINDOW { self.head = 0; }
        //fp accumulation can drift slightly negative on long silences; clamp
        let m = self.sum / NG_RMS_WINDOW as f32;
        if m < 0.0 { 0.0 } else { m }
    }
}

pub struct NoiseGate {
    mrms: MovingRms,
    threshold_sq: f32,
    hold_time_s: f32,
    sample_time_s: f32,
    attack_coeff: f32,
    release_coeff: f32,
    attack_counter: f32,
    release_counter: f32,
    smoothed_gain: f32,
}

impl NoiseGate {
    pub fn new(fs: f32, threshold: f32, attack_ms: f32, release_ms: f32, hold_ms: f32) -> Self {
        let mut ng = Self {
            mrms: MovingRms::new(),
            threshold_sq: threshold * threshold,
            hold_time_s: hold_ms * 0.001,
            sample_time_s: 1.0 / fs,
            attack_coeff: 0.0,
            release_coeff: 0.0,
            attack_counter: 0.0,
            release_counter: 0.0,
            smoothed_gain: 0.0,
        };
        ng.set_times(fs, attack_ms, release_ms);
        ng
    }

    pub fn set_threshold(&mut self, threshold: f32) {
        self.threshold_sq = threshold * threshold;
    }

    pub fn set_hold(&mut self, hold_ms: f32) {
        self.hold_time_s = hold_ms * 0.001;
    }

    //magic 2197.22... = -ln(0.0001)*1000, "time to reach 99.99% of target" form
    pub fn set_times(&mut self, fs: f32, attack_ms: f32, release_ms: f32) {
        self.attack_coeff  = (-2197.22457734 / (fs * attack_ms)).exp();
        self.release_coeff = (-2197.22457734 / (fs * release_ms)).exp();
    }

    pub fn process(&mut self, buf: &mut [f32]) {
        for sample in buf.iter_mut() {
            let in_rms_sq = self.mrms.update(*sample);
            let gain = if in_rms_sq < self.threshold_sq { 0.0 } else { 1.0 };

            if gain <= self.smoothed_gain {
                //ATTACK (signal dropped, gate engaging toward 0)
                if self.attack_counter > self.hold_time_s {
                    self.smoothed_gain = self.attack_coeff * self.smoothed_gain
                        + (1.0 - self.attack_coeff) * gain;
                } else {
                    self.attack_counter += self.sample_time_s;
                }
                self.release_counter = 0.0;
            } else {
                //RELEASE (signal returned, gate opening toward 1)
                if self.release_counter > self.hold_time_s {
                    self.smoothed_gain = self.release_coeff * self.smoothed_gain
                        + (1.0 - self.release_coeff) * gain;
                } else {
                    self.release_counter += self.sample_time_s;
                }
                self.attack_counter = 0.0;
            }

            *sample *= self.smoothed_gain;
        }
    }
}
