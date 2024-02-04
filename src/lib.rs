use embedded_hal::{delay::DelayNs, i2c::I2c};

const BQ34Z100_G1_ADDRESS: u8 = 0x55;

//
//  bq34z100g1.cpp
//  SMC
//
//  Created by Empire-Phoenix,
//  directly ported from
//  https://github.com/xkam1x/BQ34Z100G1/blob/master/bq34z100g1.cpp by Kamran Ahmad on 08/05/2019.
//  Xemics conversion from https://github.com/Ralim/BQ34Z100/blob/master/bq34z100.cpp

fn xemics_to_double(x: u32) -> f32 {
    let mut b_is_positive = false;
    let f_exponent: f32;
    let mut f_result: f32;
    let v_msbyte: u8 = (x >> 24) as u8;
    let mut v_mid_hi_byte: u8 = (x >> 16) as u8;
    let v_mid_lo_byte: u8 = (x >> 8) as u8;
    let v_lsbyte: u8 = x as u8;
    // Get the sign, its in the 0x00 80 00 00 bit
    if (v_mid_hi_byte & 128) == 0 {
        b_is_positive = true;
    }

    // Get the exponent, it's 2^(MSbyte - 0x80)
    f_exponent = 2.0_f32.powf(((v_msbyte as i16) - 128) as f32);
    println!("f_exponent {}", f_exponent);
    // Or in 0x80 to the MidHiByte
    v_mid_hi_byte = (v_mid_hi_byte | 128) as u8;
    // get value out of midhi byte
    f_result = (v_mid_hi_byte as f32) * 65536.0;
    // add in midlow byte
    f_result = f_result + (v_mid_lo_byte as f32 * 256.0) as f32;
    // add in LS byte
    f_result = f_result + v_lsbyte as f32;
    // multiply by 2^-24 to get the actual fraction
    f_result = f_result * 2.0_f32.powf(-24.0);
    // multiply fraction by the ‘exponent’ part
    f_result = f_result * f_exponent;
    // Make negative if necessary
    if b_is_positive {
        return f_result;
    } else {
        return -f_result;
    }
}

fn float_to_xemics(mut x: f32) -> u32 {
    let mut b_negative = false;

    // Vermeidung von Logarithmus von Null
    if x == 0.0 {
        x = 0.00001;
    }

    // Überprüfung auf negative Zahl
    if x < 0.0 {
        b_negative = true;
        x = -x;
    }

    // Korrekten Exponenten finden
    let i_exp = (x.log2().floor() + 1.0) as i32;

    // MS-Byte ist der Exponent + 0x80
    let i_byte1 = (i_exp + 128) as u32;

    // Eingabe durch diesen Exponenten teilen, um Mantisse zu erhalten
    let f_mantissa = x / 2f32.powi(i_exp);

    // Skalierung
    let scaled_mantissa = f_mantissa * 2f32.powi(24);

    // Aufteilung der Mantisse in 3 Bytes
    let i_byte2 = ((scaled_mantissa / 65536.0) as u32) & 0xFF;
    let i_byte3 = ((scaled_mantissa / 256.0) as u32) & 0xFF;
    let i_byte4 = (scaled_mantissa as u32) & 0xFF;

    // Subtraktion des Vorzeichenbits, falls die Zahl positiv ist
    let i_byte2 = if !b_negative { i_byte2 & 0x7F } else { i_byte2 };

    // Zusammenbau des Ergebnisses
    return (i_byte1 << 24) | (i_byte2 << 16) | (i_byte3 << 8) | i_byte4;
}

#[derive(Debug)]
pub enum Bq34Z100Error<E> {
    NotStored,
    I2C { error: E },
}

impl<E> From<E> for Bq34Z100Error<E> {
    fn from(value: E) -> Self {
        Bq34Z100Error::I2C { error: value }
    }
}

impl<I2C, DELAY, E: std::fmt::Debug> Bq34z100g1<E> for Bq34z100g1Driver<I2C, DELAY>
where
    I2C: I2c<Error = E>,
    DELAY: DelayNs,
{
    fn read_2_register_as_u16(&mut self, address: u8) -> Result<u16, Bq34Z100Error<E>> {
        let data: [u8; 1] = [address];
        let mut buffer: [u8; 2] = [0; 2];
        self.i2c
            .write_read(BQ34Z100_G1_ADDRESS, &data, &mut buffer)?;
        Ok(u16::from_le_bytes([buffer[0], buffer[1]]))
    }

    fn read_1_register_as_u8(&mut self, address: u8) -> Result<u8, Bq34Z100Error<E>> {
        let data: [u8; 1] = [address];
        let mut buffer: [u8; 1] = [0; 1];
        self.i2c
            .write_read(BQ34Z100_G1_ADDRESS, &data, &mut buffer)?;
        Ok(buffer[0])
    }

    fn read_control(&mut self, address_lsb: u8, address_msb: u8) -> Result<u16, Bq34Z100Error<E>> {
        let data: [u8; 3] = [0x00_u8, address_lsb, address_msb];
        self.i2c.write(BQ34Z100_G1_ADDRESS, &data)?;
        return self.read_2_register_as_u16(0x00);
    }

    fn internal_temperature(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_2_register_as_u16(0x2a);
    }

    fn read_flash_block(&mut self, sub_class: u8, offset: u8) -> Result<(), Bq34Z100Error<E>> {
        println!("Prepare reading block {}", sub_class);
        self.write_reg(0x61, 0x00)?; // Block control
        self.write_reg(0x3e, sub_class)?; // Flash class
        self.write_reg(0x3f, offset / 32)?; // Flash block

        let data: [u8; 1] = [0x40];
        self.i2c.write(BQ34Z100_G1_ADDRESS, &data)?;
        println!("Reading block {} now", sub_class);
        self.i2c
            .read(BQ34Z100_G1_ADDRESS, &mut self.flash_block_data)?;
        return Ok(());
    }

    fn write_reg(&mut self, address: u8, value: u8) -> Result<(), Bq34Z100Error<E>> {
        let data: [u8; 2] = [address, value];
        self.i2c.write(BQ34Z100_G1_ADDRESS, &data)?;

        println!(
            "Writing register block {:#04x} with value {}",
            address, value
        );
        return Ok(());
    }

    fn write_flash_block(&mut self, sub_class: u8, offset: u8) -> Result<(), Bq34Z100Error<E>> {
        self.write_reg(0x61, 0x00)?; // Block control
        self.write_reg(0x3e, sub_class)?; // Flash class
        self.write_reg(0x3f, offset / 32)?; // Flash block

        self.i2c
            .write(BQ34Z100_G1_ADDRESS, &self.flash_block_data)?;
        return Ok(());
    }

    fn flash_block_checksum(&mut self) -> Result<u8, Bq34Z100Error<E>> {
        let mut temp: u8 = 0;
        for i in self.flash_block_data.iter() {
            temp = u8::wrapping_add(temp, *i);
        }
        return Ok(u8::wrapping_sub(255, temp));
    }

    fn unsealed(&mut self) -> Result<(), Bq34Z100Error<E>> {
        println!("Unsealing");
        let data: [u8; 3] = [0x00, 0x14, 0x04];
        self.i2c.write(BQ34Z100_G1_ADDRESS, &data)?;

        let data2: [u8; 3] = [0x00, 0x72, 0x36];
        self.i2c.write(BQ34Z100_G1_ADDRESS, &data2)?;
        return Ok(());
    }

    fn enter_calibration(&mut self) -> Result<(), Bq34Z100Error<E>> {
        println!("enter_calibration");
        self.unsealed()?;
        loop {
            self.cal_enable()?;
            println!("Enable cal");
            self.enter_cal()?;
            self.delay.delay_ms(1000);
            if self.control_status()? & 0x1000 > 0 {
                break;
            }
        } // CALEN
        return Ok(());
    }

    fn exit_calibration(&mut self) -> Result<(), Bq34Z100Error<E>> {
        loop {
            self.exit_cal()?;
            self.delay.delay_ms(1000);
            if self.control_status()? & 0x1000 == 0 {
                break;
            }
        } // CALEN
        self.delay.delay_ms(150);
        self.reset()?;
        self.delay.delay_ms(150);
        return Ok(());
    }

    /**
     * If you are using a lipo li-ion battery this should be the only one you use. Since you cannot change the chemid with this driver,
     * and need to use the BatteryManager desktop application anyway, I strongly recommend to set all other config there as well.
     */
    fn update_design_capacity(&mut self, capacity: u16) -> Result<(), Bq34Z100Error<E>> {
        self.unsealed()?;
        self.read_flash_block(48, 0)?;

        self.flash_block_data[6] = 0; // Cycle Count
        self.flash_block_data[7] = 0;

        self.flash_block_data[8] = (capacity >> 8) as u8; // CC Threshold
        self.flash_block_data[9] = (capacity & 0xff) as u8;

        self.flash_block_data[11] = (capacity >> 8) as u8; // Design Capacity
        self.flash_block_data[12] = (capacity & 0xff) as u8;

        println!(
            "Block 11 {} block 12 {}",
            self.flash_block_data[11], self.flash_block_data[12]
        );

        for i in 6..=9 {
            self.write_reg(0x40 + i, self.flash_block_data[i as usize])?;
        }

        for i in 11..=12 {
            self.write_reg(0x40 + i, self.flash_block_data[i as usize])?;
        }

        let checksum = self.flash_block_checksum()?;
        self.write_reg(0x60, checksum)?;

        println!("Checksum {}", checksum);

        self.delay.delay_ms(150);
        self.reset()?;
        self.delay.delay_ms(150);

        self.unsealed()?;

        self.read_flash_block(48, 0)?;
        let mut updated_cc_threshold: i16 = (self.flash_block_data[8] as i16) << 8_i16;
        updated_cc_threshold |= self.flash_block_data[9] as i16;

        let mut updated_capacity: i16 = (self.flash_block_data[11] as i16) << 8;
        updated_capacity |= self.flash_block_data[12] as i16;

        if self.flash_block_data[6] != 0 || self.flash_block_data[7] != 0 {
            println!("Block 6 or 7 wrong");
            return Err(Bq34Z100Error::NotStored);
        }
        println!(
            "Expected capacity {} updated threshold {}",
            capacity, updated_capacity
        );
        if capacity as i32 != updated_cc_threshold as i32 {
            println!("cc threshold wrong");
            return Err(Bq34Z100Error::NotStored);
        }
        if capacity as i32 != updated_capacity as i32 {
            println!("capacity wrong");
            return Err(Bq34Z100Error::NotStored);
        }
        return Ok(());
    }

    fn update_q_max(&mut self, capacity: i16) -> Result<(), Bq34Z100Error<E>> {
        self.unsealed()?;
        self.read_flash_block(82, 0)?;
        self.flash_block_data[0] = (capacity >> 8) as u8; // Q Max
        self.flash_block_data[1] = (capacity & 0xff) as u8;

        self.flash_block_data[2] = 0; // Cycle Count
        self.flash_block_data[3] = 0;

        for i in 0_u8..3_u8 {
            self.write_reg(0x40 + i, self.flash_block_data[i as usize])?;
        }

        let checksum = self.flash_block_checksum()?;
        self.write_reg(0x60, checksum)?;

        self.delay.delay_ms(150);
        self.reset()?;
        self.delay.delay_ms(150);

        self.unsealed()?;
        self.read_flash_block(82, 0)?;
        let mut updated_q_max: i16 = (self.flash_block_data[0] as i16) << 8;
        updated_q_max |= self.flash_block_data[1] as i16;

        if capacity != updated_q_max {
            return Err(Bq34Z100Error::NotStored);
        }
        return Ok(());
    }

    /**
     * If you are using a lipo li-ion battery this should be the only one you use. Since you cannot change the chemid with this driver,
     * and need to use the BatteryManager desktop application anyway, I strongly recommend to set all other config there as well.
     */
    fn update_design_energy(
        &mut self,
        energy: i16,
        energy_scale: u8,
    ) -> Result<(), Bq34Z100Error<E>> {
        self.unsealed()?;
        self.read_flash_block(48, 0)?;
        self.flash_block_data[13] = (energy >> 8) as u8; // Design Energy
        self.flash_block_data[14] = (energy & 0xff) as u8;
        self.flash_block_data[30] = energy_scale;

        for i in 13..=14 {
            self.write_reg(0x40 + i, self.flash_block_data[i as usize])?;
        }
        self.write_reg(0x40 + 30, self.flash_block_data[30])?;

        let checksum = self.flash_block_checksum()?;
        self.write_reg(0x60, checksum)?;

        self.delay.delay_ms(150);
        self.reset()?;
        self.delay.delay_ms(150);

        self.unsealed()?;
        self.read_flash_block(48, 0)?;
        let mut updated_energy: i16 = (self.flash_block_data[13] as i16) << 8;
        updated_energy |= self.flash_block_data[14] as i16;

        if energy != updated_energy {
            return Err(Bq34Z100Error::NotStored);
        }

        let updated_energy_scale: u8 = self.flash_block_data[30];
        if updated_energy_scale != energy_scale {
            return Err(Bq34Z100Error::NotStored);
        }

        return Ok(());
    }

    fn update_cell_charge_voltage_range(
        &mut self,
        t1_t2: u16,
        t2_t3: u16,
        t3_t4: u16,
    ) -> Result<(), Bq34Z100Error<E>> {
        self.unsealed()?;
        self.read_flash_block(48, 0)?;

        self.flash_block_data[17] = (t1_t2 >> 8) as u8; // Cell Charge Voltage T1-T2
        self.flash_block_data[18] = (t1_t2 & 0xff) as u8;

        self.flash_block_data[19] = (t2_t3 >> 8) as u8; // Cell Charge Voltage T2-T3
        self.flash_block_data[20] = (t2_t3 & 0xff) as u8;

        self.flash_block_data[21] = (t3_t4 >> 8) as u8; // Cell Charge Voltage T3-T4
        self.flash_block_data[22] = (t3_t4 & 0xff) as u8;

        for i in 17..=22 {
            self.write_reg(0x40 + i, self.flash_block_data[i as usize])?;
        }

        let checksum = self.flash_block_checksum()?;
        self.write_reg(0x60, checksum)?;

        self.delay.delay_ms(150);
        self.reset()?;
        self.delay.delay_ms(150);

        self.unsealed()?;
        self.read_flash_block(48, 0)?;
        let mut updated_t1_t2: u16 = (self.flash_block_data[17] as u16) << 8;
        updated_t1_t2 |= self.flash_block_data[18] as u16;

        let mut updated_t2_t3: u16 = (self.flash_block_data[19] as u16) << 8;
        updated_t2_t3 |= self.flash_block_data[20] as u16;

        let mut updated_t3_t4: u16 = (self.flash_block_data[21] as u16) << 8;
        updated_t3_t4 |= self.flash_block_data[22] as u16;

        if t1_t2 as u16 != updated_t1_t2
            || t2_t3 as u16 != updated_t2_t3
            || t3_t4 as u16 != updated_t3_t4
        {
            return Err(Bq34Z100Error::NotStored);
        }
        return Ok(());
    }

    fn set_led_mode(&mut self, led_config: u8) -> Result<(), Bq34Z100Error<E>> {
        self.unsealed()?;
        self.read_flash_block(64, 0)?;
        self.flash_block_data[4] = led_config;
        self.write_reg(0x40 + 4, self.flash_block_data[4])?;

        let checksum = self.flash_block_checksum()?;
        self.write_reg(0x60, checksum)?;

        self.delay.delay_ms(150);
        self.reset()?;
        self.delay.delay_ms(150);

        self.unsealed()?;
        self.read_flash_block(64, 0)?;

        if self.flash_block_data[4] != led_config {
            println!("Failed to set led config!");
            return Err(Bq34Z100Error::NotStored);
        }
        return Ok(());
    }

    /**
     * If you are using a lipo li-ion battery this should be the only one you use. Since you cannot change the chemid with this driver,
     * and need to use the BatteryManager desktop application anyway, I strongly recommend to set all other config there as well.
     */
    fn update_number_of_series_cells(&mut self, cells: u8) -> Result<(), Bq34Z100Error<E>> {
        self.unsealed()?;
        self.read_flash_block(64, 0)?;

        self.flash_block_data[7] = cells; // Number of Series Cell
        self.write_reg(0x40 + 7, self.flash_block_data[7])?;

        let checksum = self.flash_block_checksum()?;
        self.write_reg(0x60, checksum)?;

        self.delay.delay_ms(150);
        self.reset()?;
        self.delay.delay_ms(150);

        self.unsealed()?;
        self.read_flash_block(64, 0)?;

        if cells != self.flash_block_data[7] {
            return Err(Bq34Z100Error::NotStored);
        }
        return Ok(());
    }

    /**
     * If you are using a lipo li-ion battery this should be the only one you use. Since you cannot change the chemid with this driver,
     * and need to use the BatteryManager desktop application anyway, I strongly recommend to set all other config there as well.
     */
    fn update_pack_configuration(&mut self, config: u16) -> Result<(), Bq34Z100Error<E>> {
        self.unsealed()?;
        self.read_flash_block(64, 0)?;

        self.flash_block_data[0] = (config >> 8) as u8; // Pack Configuration
        self.flash_block_data[1] = (config & 0xff) as u8;

        for i in 0..=1 {
            self.write_reg(0x40 + i, self.flash_block_data[i as usize])?;
        }

        let checksum = self.flash_block_checksum()?;
        self.write_reg(0x60, checksum)?;

        self.delay.delay_ms(150);
        self.reset()?;
        self.delay.delay_ms(1000);

        self.unsealed()?;
        self.read_flash_block(64, 0)?;
        let mut updated_config = (self.flash_block_data[0] as u16) << 8;
        updated_config |= self.flash_block_data[1] as u16;
        if config != updated_config {
            return Err(Bq34Z100Error::NotStored);
        }
        return Ok(());
    }

    //Not recommended to use this
    fn update_charge_termination_parameters(
        &mut self,
        taper_current: i16,
        min_taper_capacity: i16,
        cell_taper_voltage: i16,
        taper_window: u8,
        tca_set: i8,
        tca_clear: i8,
        fc_set: i8,
        fc_clear: i8,
    ) -> Result<(), Bq34Z100Error<E>> {
        self.unsealed()?;
        self.read_flash_block(36, 0)?;

        self.flash_block_data[0] = (taper_current >> 8) as u8; // Taper Current
        self.flash_block_data[1] = (taper_current & 0xff) as u8;

        self.flash_block_data[2] = (min_taper_capacity >> 8) as u8; // Min Taper Capacity
        self.flash_block_data[3] = (min_taper_capacity & 0xff) as u8;

        self.flash_block_data[4] = (cell_taper_voltage >> 8) as u8; // Cell Taper Voltage
        self.flash_block_data[5] = (cell_taper_voltage & 0xff) as u8;

        self.flash_block_data[6] = taper_window; // Current Taper Window

        self.flash_block_data[7] = (tca_set as u8) & 0xff; // TCA Set %

        self.flash_block_data[8] = (tca_clear as u8) & 0xff; // TCA Clear %

        self.flash_block_data[9] = (fc_set as u8) & 0xff; // FC Set %

        self.flash_block_data[10] = (fc_clear as u8) & 0xff; // FC Clear %

        for i in 0..=10 {
            self.write_reg(0x40 + i, self.flash_block_data[i as usize])?;
        }

        let checksum = self.flash_block_checksum()?;
        self.write_reg(0x60, checksum)?;

        self.delay.delay_ms(150);
        self.reset()?;
        self.delay.delay_ms(150);

        self.unsealed()?;
        self.read_flash_block(36, 0)?;
        let mut updated_taper_current: i16;
        let mut updated_min_taper_capacity: i16;
        let mut updated_cell_taper_voltage: i16;
        let updated_taper_window: u8;
        let updated_tca_set: i8;
        let updated_tca_clear: i8;
        let updated_fc_set: i8;
        let updated_fc_clear: i8;

        updated_taper_current = (self.flash_block_data[0] as i16) << 8;
        updated_taper_current |= self.flash_block_data[1] as i16;

        updated_min_taper_capacity = (self.flash_block_data[2] as i16) << 8;
        updated_min_taper_capacity |= self.flash_block_data[3] as i16;

        updated_cell_taper_voltage = (self.flash_block_data[4] as i16) << 8;
        updated_cell_taper_voltage |= self.flash_block_data[5] as i16;

        updated_taper_window = self.flash_block_data[6];

        updated_tca_set = (self.flash_block_data[7] & 0xff) as i8;

        updated_tca_clear = (self.flash_block_data[8] & 0xff) as i8;

        updated_fc_set = (self.flash_block_data[9] & 0xff) as i8;

        updated_fc_clear = (self.flash_block_data[10] & 0xff) as i8;

        if taper_current != updated_taper_current {
            println!(
                "Could not update taper current expected {} actual {}",
                taper_current, updated_taper_current
            );
            return Err(Bq34Z100Error::NotStored);
        }
        if min_taper_capacity != updated_min_taper_capacity {
            println!(
                "Could not update min_taper_capacity expected {} actual {}",
                min_taper_capacity, updated_min_taper_capacity
            );
            return Err(Bq34Z100Error::NotStored);
        }
        if cell_taper_voltage != updated_cell_taper_voltage {
            println!(
                "Could not update cell_taper_voltage expected {} actual {}",
                cell_taper_voltage, updated_cell_taper_voltage
            );
            return Err(Bq34Z100Error::NotStored);
        }
        if taper_window != updated_taper_window {
            println!(
                "Could not update taper_window expected {} actual {}",
                taper_window, updated_taper_window
            );
            return Err(Bq34Z100Error::NotStored);
        }
        if tca_set != updated_tca_set {
            println!(
                "Could not update tca_set expected {} actual {}",
                tca_set, updated_tca_set
            );
            return Err(Bq34Z100Error::NotStored);
        }
        if tca_clear != updated_tca_clear {
            println!(
                "Could not update tca_clear expected {} actual {}",
                tca_clear, updated_tca_clear
            );
            return Err(Bq34Z100Error::NotStored);
        }
        if fc_set != updated_fc_set {
            println!(
                "Could not update fc_set expected {} actual {}",
                fc_set, updated_fc_set
            );
            return Err(Bq34Z100Error::NotStored);
        }
        if fc_clear != updated_fc_clear {
            println!(
                "Could not update fc_clear expected {} actual {}",
                fc_clear, updated_fc_clear
            );
            return Err(Bq34Z100Error::NotStored);
        }
        return Ok(());
    }

    fn calibrate_cc_offset(&mut self) -> Result<(), Bq34Z100Error<E>> {
        self.enter_calibration()?;

        loop {
            println!("Loop cc offset");
            self.cc_offset()?;
            self.delay.delay_ms(1000);
            if self.control_status()? & 0x0800 > 0 {
                break;
            }
        } // CCA

        loop {
            self.delay.delay_ms(1000);
            if self.control_status()? & 0x0800 == 0 {
                break;
            }
        } // CCA

        self.cc_offset_save()?;
        self.exit_calibration()?;
        return Ok(());
    }

    fn calibrate_board_offset(&mut self) -> Result<(), Bq34Z100Error<E>> {
        self.enter_calibration()?;
        loop {
            self.board_offset()?;
            self.delay.delay_ms(1000);
            if self.control_status()? & 0x0c00 > 0 {
                break;
            }
        } // CCA + BCA

        loop {
            self.delay.delay_ms(1000);
            if self.control_status()? & 0x0c00 == 0 {
                break;
            }
        } // CCA + BCA

        self.cc_offset_save()?;
        self.exit_calibration()?;
        return Ok(());
    }

    fn calibrate_voltage_divider(&mut self, applied_voltage: f32) -> Result<(), Bq34Z100Error<E>> {
        let mut volt_array: [f32; 50] = [0.0; 50];
        for i in 0..50 {
            volt_array[i] = self.voltage()? as f32;
            self.delay.delay_ms(150);
            println!("Reading voltage {} as {}", i, volt_array[i]);
        }
        let mut volt_mean: f32 = 0.0;
        for i in 0..50 {
            volt_mean += volt_array[i];
        }
        volt_mean /= 50.0;

        let mut volt_sd: f32 = 0.0;
        for i in 0..50 {
            volt_sd += (volt_array[i] - volt_mean).powf(2.0);
        }
        volt_sd /= 50.0;
        volt_sd = volt_sd.sqrt();

        if volt_sd > 100.0 {
            return Ok(());
        }

        self.unsealed()?;

        self.read_flash_block(104, 0)?;

        let mut current_voltage_divider: u16 = (self.flash_block_data[14] as u16) << 8;
        current_voltage_divider |= self.flash_block_data[15] as u16;

        let new_voltage_divider: u16 =
            ((applied_voltage as f32 / volt_mean as f32) * current_voltage_divider as f32) as u16;

        println!("Setting new voltage divider to {}", new_voltage_divider);

        self.flash_block_data[14] = (new_voltage_divider >> 8) as u8;
        self.flash_block_data[15] = (new_voltage_divider & 0xff) as u8;

        for i in 14..=15 {
            self.write_reg(0x40 + i, self.flash_block_data[i as usize])?;
        }

        let checksum = self.flash_block_checksum()?;
        self.write_reg(0x60, checksum)?;
        self.delay.delay_ms(150);
        return Ok(());
    }

    fn calibrate_sense_resistor(&mut self, applied_current: i16) -> Result<(), Bq34Z100Error<E>> {
        let mut current_array: [f32; 50] = [0.0; 50];
        for i in 0..50 {
            current_array[i] = self.current()? as f32;
            println!("Reading current {} @ {}", current_array[i], i);
            self.delay.delay_ms(150);
        }
        let mut current_mean: f32 = 0.0;
        for i in 0..50 {
            current_mean += current_array[i];
        }
        current_mean /= 50.0;

        let mut current_sd: f32 = 0.0;
        for i in 0..50 {
            current_sd += (current_array[i] - current_mean).powf(2.0);
        }
        current_sd /= 50.0;
        current_sd = current_sd.sqrt();

        if current_sd > 100.0 {
            //actually this might not be ok?
            return Ok(());
        }

        self.unsealed()?;
        self.read_flash_block(104, 0)?;

        let mut cc_gain: u32 = (self.flash_block_data[0] as u32) << 24;
        cc_gain |= (self.flash_block_data[1] as u32) << 16;
        cc_gain |= (self.flash_block_data[2] as u32) << 8;
        cc_gain |= self.flash_block_data[3] as u32;

        let float_cc_gain = xemics_to_double(cc_gain);
        let xemics_cc_gain = float_to_xemics(float_cc_gain);
        let float_cc_gain2 = xemics_to_double(xemics_cc_gain);
        if (float_cc_gain - float_cc_gain2).abs() > 0.01 {
            println!("Error converting old gain!!");
        }

        let gain_resistence: f32 = 4.768 / float_cc_gain;
        println!(
            "Current gain R is {}  xemics is {}",
            gain_resistence, cc_gain
        );

        let temp: f32 = (current_mean * gain_resistence) / applied_current as f32;
        println!(
            "Current is {} , applied current ist {}, new gain is {}",
            current_mean, applied_current, temp
        );

        let mut new_cc_gain: u32 = float_to_xemics(4.768 / temp);
        self.flash_block_data[0] = (new_cc_gain >> 24) as u8;
        self.flash_block_data[1] = (new_cc_gain >> 16) as u8;
        self.flash_block_data[2] = (new_cc_gain >> 8) as u8;
        self.flash_block_data[3] = (new_cc_gain & 0xff) as u8;

        new_cc_gain = float_to_xemics(5677445.6 / temp);
        self.flash_block_data[4] = (new_cc_gain >> 24) as u8;
        self.flash_block_data[5] = (new_cc_gain >> 16) as u8;
        self.flash_block_data[6] = (new_cc_gain >> 8) as u8;
        self.flash_block_data[7] = (new_cc_gain & 0xff) as u8;

        for i in 0..=3 {
            self.write_reg(0x40 + i, self.flash_block_data[i as usize])?;
        }

        for i in 4..=7 {
            self.write_reg(0x40 + i, self.flash_block_data[i as usize])?;
        }

        let checksum = self.flash_block_checksum()?;
        self.write_reg(0x60, checksum)?;
        self.delay.delay_ms(150);
        self.reset()?;
        self.delay.delay_ms(150);
        return Ok(());
    }

    fn set_current_deadband(&mut self, deadband: u8) -> Result<(), Bq34Z100Error<E>> {
        let _ = deadband;
        todo!()
    }

    fn ready(&mut self) -> Result<(), Bq34Z100Error<E>> {
        self.unsealed()?;
        self.it_enable()?;
        return Ok(());
    }

    fn control_status(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_control(0x00, 0x00);
    }

    fn device_type(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_control(0x01, 0x00);
    }

    fn fw_version(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_control(0x02, 0x00);
    }

    fn hw_version(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_control(0x03, 0x00);
    }

    fn reset_data(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_control(0x05, 0x00);
    }

    fn prev_macwrite(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_control(0x07, 0x00);
    }

    fn chem_id(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_control(0x08, 0x00);
    }

    fn board_offset(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_control(0x09, 0x00);
    }

    fn cc_offset(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_control(0x0a, 0x00);
    }

    fn cc_offset_save(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_control(0x0b, 0x00);
    }

    fn df_version(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_control(0x0c, 0x00);
    }

    fn set_fullsleep(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_control(0x10, 0x00);
    }

    fn static_chem_chksum(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_control(0x17, 0x00);
    }

    fn sealed(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        //why do we want to ever seal a hobby diy project tho? anyway pull requests welcome ;)
        todo!()
    }

    fn it_enable(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_control(0x21, 0x00);
    }

    fn cal_enable(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_control(0x2d, 0x00);
    }

    fn reset(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_control(0x41, 0x00);
    }

    fn exit_cal(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_control(0x80, 0x00);
    }

    fn enter_cal(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_control(0x81, 0x00);
    }

    fn offset_cal(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_control(0x82, 0x00);
    }

    fn state_of_charge(&mut self) -> Result<u8, Bq34Z100Error<E>> {
        return self.read_1_register_as_u8(0x02);
    }

    fn state_of_charge_max_error(&mut self) -> Result<u8, Bq34Z100Error<E>> {
        return self.read_1_register_as_u8(0x03);
    }

    fn remaining_capacity(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_2_register_as_u16(0x04);
    }

    fn full_charge_capacity(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_2_register_as_u16(0x06);
    }

    fn voltage(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_2_register_as_u16(0x08);
    }

    fn average_current(&mut self) -> Result<i16, Bq34Z100Error<E>> {
        return Ok(self.read_2_register_as_u16(0x0a)? as i16);
    }

    fn temperature(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_2_register_as_u16(0x0c);
    }

    fn flags(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_2_register_as_u16(0x0e);
    }

    fn flags_b(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_2_register_as_u16(0x12);
    }

    fn current(&mut self) -> Result<i16, Bq34Z100Error<E>> {
        return Ok(self.read_2_register_as_u16(0x10)? as i16);
    }

    fn average_time_to_empty(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_2_register_as_u16(0x18);
    }

    fn average_time_to_full(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_2_register_as_u16(0x1a);
    }

    fn passed_charge(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_2_register_as_u16(0x1c);
    }

    fn do_d0_time(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_2_register_as_u16(0x1e);
    }

    fn available_energy(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_2_register_as_u16(0x24);
    }

    fn average_power(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_2_register_as_u16(0x26);
    }

    fn serial_number(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_2_register_as_u16(0x28);
    }

    fn cycle_count(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_2_register_as_u16(0x2c);
    }

    fn state_of_health(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_2_register_as_u16(0x2e);
    }

    fn charge_voltage(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_2_register_as_u16(0x30);
    }

    fn charge_current(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_2_register_as_u16(0x32);
    }

    fn pack_configuration(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_2_register_as_u16(0x3a);
    }

    fn design_capacity(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_2_register_as_u16(0x3c);
    }

    fn grid_number(&mut self) -> Result<u8, Bq34Z100Error<E>> {
        return self.read_1_register_as_u8(0x62);
    }

    fn learned_status(&mut self) -> Result<u8, Bq34Z100Error<E>> {
        return self.read_1_register_as_u8(0x63);
    }

    fn dod_at_eoc(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_2_register_as_u16(0x64);
    }

    fn q_start(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_2_register_as_u16(0x66);
    }

    fn true_fcc(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_2_register_as_u16(0x6a);
    }

    fn state_time(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_2_register_as_u16(0x6c);
    }

    fn q_max_passed_q(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_2_register_as_u16(0x6e);
    }

    fn dod_0(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_2_register_as_u16(0x70);
    }

    fn q_max_dod_0(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_2_register_as_u16(0x72);
    }

    fn q_max_time(&mut self) -> Result<u16, Bq34Z100Error<E>> {
        return self.read_2_register_as_u16(0x74);
    }

    fn get_flags_decoded(&mut self) -> Result<Flags, Bq34Z100Error<E>> {
        let flags = self.flags()?.to_le_bytes();

        return Ok(Flags {
            fast_charge_allowed: flags[0] >> 0 & 1 > 0,
            full_chage: flags[0] >> 1 & 1 > 0,
            charging_not_allowed: flags[0] >> 2 & 1 > 0,
            charge_inhibit: flags[0] >> 3 & 1 > 0,
            bat_low: flags[0] >> 4 & 1 > 0,
            bat_high: flags[0] >> 5 & 1 > 0,
            over_temp_discharge: flags[0] >> 6 & 1 > 0,
            over_temp_charge: flags[0] >> 7 & 1 > 0,

            discharge: flags[1] >> 0 & 1 > 0,
            state_of_charge_f: flags[1] >> 1 & 1 > 0,
            state_of_charge_1: flags[1] >> 2 & 1 > 0,
            cf: flags[1] >> 4 & 1 > 0,
            ocv_taken: flags[1] >> 7 & 1 > 0,
        });
    }
}

pub struct Bq34z100g1Driver<I2C, Delay> {
    pub i2c: I2C,
    pub delay: Delay,
    pub flash_block_data: [u8; 32],
}
pub trait Bq34z100g1<E> {
    fn read_2_register_as_u16(&mut self, address: u8) -> Result<u16, Bq34Z100Error<E>>;
    fn read_1_register_as_u8(&mut self, address: u8) -> Result<u8, Bq34Z100Error<E>>;
    fn read_control(&mut self, address_lsb: u8, address_msb: u8) -> Result<u16, Bq34Z100Error<E>>;
    fn read_flash_block(&mut self, sub_class: u8, offset: u8) -> Result<(), Bq34Z100Error<E>>;
    fn write_reg(&mut self, address: u8, value: u8) -> Result<(), Bq34Z100Error<E>>;
    fn write_flash_block(&mut self, sub_class: u8, offset: u8) -> Result<(), Bq34Z100Error<E>>;

    fn flash_block_checksum(&mut self) -> Result<u8, Bq34Z100Error<E>>;

    fn unsealed(&mut self) -> Result<(), Bq34Z100Error<E>>;
    fn enter_calibration(&mut self) -> Result<(), Bq34Z100Error<E>>;
    fn exit_calibration(&mut self) -> Result<(), Bq34Z100Error<E>>;

    fn update_design_capacity(&mut self, capacity: u16) -> Result<(), Bq34Z100Error<E>>;
    fn update_q_max(&mut self, capacity: i16) -> Result<(), Bq34Z100Error<E>>;
    fn update_design_energy(&mut self, energy: i16, scale: u8) -> Result<(), Bq34Z100Error<E>>;
    fn update_cell_charge_voltage_range(
        &mut self,
        t1_t2: u16,
        t2_t3: u16,
        t3_t4: u16,
    ) -> Result<(), Bq34Z100Error<E>>;
    fn update_number_of_series_cells(&mut self, cells: u8) -> Result<(), Bq34Z100Error<E>>;
    fn update_pack_configuration(&mut self, config: u16) -> Result<(), Bq34Z100Error<E>>;
    fn update_charge_termination_parameters(
        &mut self,
        taper_current: i16,
        min_taper_capacity: i16,
        cell_taper_voltage: i16,
        taper_window: u8,
        tca_set: i8,
        tca_clear: i8,
        fc_set: i8,
        fc_clear: i8,
    ) -> Result<(), Bq34Z100Error<E>>;
    fn calibrate_cc_offset(&mut self) -> Result<(), Bq34Z100Error<E>>;
    fn calibrate_board_offset(&mut self) -> Result<(), Bq34Z100Error<E>>;
    fn calibrate_voltage_divider(&mut self, applied_voltage: f32) -> Result<(), Bq34Z100Error<E>>;
    fn calibrate_sense_resistor(&mut self, applied_current: i16) -> Result<(), Bq34Z100Error<E>>;
    fn set_current_deadband(&mut self, deadband: u8) -> Result<(), Bq34Z100Error<E>>;
    fn ready(&mut self) -> Result<(), Bq34Z100Error<E>>;

    fn control_status(&mut self) -> Result<u16, Bq34Z100Error<E>>;
    fn device_type(&mut self) -> Result<u16, Bq34Z100Error<E>>;
    fn fw_version(&mut self) -> Result<u16, Bq34Z100Error<E>>;
    fn hw_version(&mut self) -> Result<u16, Bq34Z100Error<E>>;
    fn reset_data(&mut self) -> Result<u16, Bq34Z100Error<E>>;
    fn prev_macwrite(&mut self) -> Result<u16, Bq34Z100Error<E>>;
    fn chem_id(&mut self) -> Result<u16, Bq34Z100Error<E>>;
    fn board_offset(&mut self) -> Result<u16, Bq34Z100Error<E>>;
    fn cc_offset(&mut self) -> Result<u16, Bq34Z100Error<E>>;
    fn cc_offset_save(&mut self) -> Result<u16, Bq34Z100Error<E>>;
    fn df_version(&mut self) -> Result<u16, Bq34Z100Error<E>>;
    fn set_fullsleep(&mut self) -> Result<u16, Bq34Z100Error<E>>;
    fn static_chem_chksum(&mut self) -> Result<u16, Bq34Z100Error<E>>;
    fn sealed(&mut self) -> Result<u16, Bq34Z100Error<E>>;
    fn it_enable(&mut self) -> Result<u16, Bq34Z100Error<E>>;
    fn cal_enable(&mut self) -> Result<u16, Bq34Z100Error<E>>;
    fn reset(&mut self) -> Result<u16, Bq34Z100Error<E>>;
    fn exit_cal(&mut self) -> Result<u16, Bq34Z100Error<E>>;
    fn enter_cal(&mut self) -> Result<u16, Bq34Z100Error<E>>;
    fn offset_cal(&mut self) -> Result<u16, Bq34Z100Error<E>>;

    fn state_of_charge(&mut self) -> Result<u8, Bq34Z100Error<E>>; // 0 to 100%
    fn state_of_charge_max_error(&mut self) -> Result<u8, Bq34Z100Error<E>>; // 1 to 100%
    fn remaining_capacity(&mut self) -> Result<u16, Bq34Z100Error<E>>; // mAh
    fn full_charge_capacity(&mut self) -> Result<u16, Bq34Z100Error<E>>; // mAh
    fn voltage(&mut self) -> Result<u16, Bq34Z100Error<E>>; // mV
    fn average_current(&mut self) -> Result<i16, Bq34Z100Error<E>>; // mA
    fn temperature(&mut self) -> Result<u16, Bq34Z100Error<E>>; // Unit of x10 K
    fn flags(&mut self) -> Result<u16, Bq34Z100Error<E>>;
    fn flags_b(&mut self) -> Result<u16, Bq34Z100Error<E>>;
    fn current(&mut self) -> Result<i16, Bq34Z100Error<E>>; // mA

    fn average_time_to_empty(&mut self) -> Result<u16, Bq34Z100Error<E>>; // Minutes
    fn average_time_to_full(&mut self) -> Result<u16, Bq34Z100Error<E>>; // Minutes
    fn passed_charge(&mut self) -> Result<u16, Bq34Z100Error<E>>; // mAh
    fn do_d0_time(&mut self) -> Result<u16, Bq34Z100Error<E>>; // Minutes
    fn available_energy(&mut self) -> Result<u16, Bq34Z100Error<E>>; // 10 mWh
    fn average_power(&mut self) -> Result<u16, Bq34Z100Error<E>>; // 10 mW
    fn serial_number(&mut self) -> Result<u16, Bq34Z100Error<E>>;
    fn internal_temperature(&mut self) -> Result<u16, Bq34Z100Error<E>>; // Unit of x10 K
    fn cycle_count(&mut self) -> Result<u16, Bq34Z100Error<E>>; // Counts
    fn state_of_health(&mut self) -> Result<u16, Bq34Z100Error<E>>; // 0 to 100%
    fn charge_voltage(&mut self) -> Result<u16, Bq34Z100Error<E>>; // mV
    fn charge_current(&mut self) -> Result<u16, Bq34Z100Error<E>>; // mA
    fn pack_configuration(&mut self) -> Result<u16, Bq34Z100Error<E>>;
    fn design_capacity(&mut self) -> Result<u16, Bq34Z100Error<E>>; // mAh
    fn grid_number(&mut self) -> Result<u8, Bq34Z100Error<E>>;
    fn learned_status(&mut self) -> Result<u8, Bq34Z100Error<E>>;
    fn dod_at_eoc(&mut self) -> Result<u16, Bq34Z100Error<E>>;
    fn q_start(&mut self) -> Result<u16, Bq34Z100Error<E>>; // mAh
    fn true_fcc(&mut self) -> Result<u16, Bq34Z100Error<E>>; // mAh
    fn state_time(&mut self) -> Result<u16, Bq34Z100Error<E>>; // s
    fn q_max_passed_q(&mut self) -> Result<u16, Bq34Z100Error<E>>; // mAh
    fn dod_0(&mut self) -> Result<u16, Bq34Z100Error<E>>;
    fn q_max_dod_0(&mut self) -> Result<u16, Bq34Z100Error<E>>;
    fn q_max_time(&mut self) -> Result<u16, Bq34Z100Error<E>>;
    fn set_led_mode(&mut self, led_config: u8) -> Result<(), Bq34Z100Error<E>>;
    fn get_flags_decoded(&mut self) -> Result<Flags, Bq34Z100Error<E>>;
}

#[derive(Debug)]
pub struct Flags {
    pub fast_charge_allowed: bool,
    pub full_chage: bool,
    pub charging_not_allowed: bool,
    pub charge_inhibit: bool,
    pub bat_low: bool,
    pub bat_high: bool,
    pub over_temp_discharge: bool,
    pub over_temp_charge: bool,
    pub discharge: bool,
    pub state_of_charge_f: bool,
    pub state_of_charge_1: bool,
    pub cf: bool,
    pub ocv_taken: bool,
}
