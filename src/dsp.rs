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
