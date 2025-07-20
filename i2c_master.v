`timescale 1fs/1fs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// NAME: G.Nikhith,V.V.N Sai                                                                                                                               /
// Create Date: 01.07.2025 22:39:02
// Design Name: I2C_master_core
// Project Name: Initiating I2C protocol in FPGA BASYS-3. 
// Description: 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
module i2c_controller(
    input             clock,                // 100mhz clk (initial clk)
    input             rst_n,                // Active-low reset signal. When low, it resets the module to its initial state.
    input      [7:0]  dev_address_rw,       // 7 bit Address and LSB-->R/W Bit 0 for Write and 1 for Read
    input      [15:0] secondary_addr,       // A 16-bit input specifying the sub-address (or register address) within the I2C slave device.
    input             sec_addr_len,         // A single-bit input that determines the size of the sub-address: 0 for an 8-bit sub-address, 1 for a 16-bit sub-address.
    input      [23:0] data_length,          // A 24-bit input specifying the number of bytes to read or write in a transaction.
    input      [7:0]  tx_data,              // An 8-bit input containing the data byte to be written to the I2C slave during a write operation.
    input             transfer_request,     // A single-bit input that triggers the start of a new I2C transaction when high.

    // For read Function
    output reg [7:0]  rx_data,              // An 8-bit register output that holds the data byte read from the I2C slave during a read operation.
    output reg        data_valid,           // A single-bit register output that indicates whether the data in rx_data is valid (1) or not (0).
    // I2C Lines
    inout             scl_line,             // The I2C clock line (SCL), which operates at 100 kHz. It's bidirectional and set to high-impedance (1'bz) when not driven.
    inout             sda_line,             // The I2C data line (SDA), also bidirectional, used for data transfer and set to high-impedance when not driven.
    // extra measures for master module
    output reg        request_data,         // A register output that signals the external system to provide new data for transmission.
    output reg        active,               // A register output that is high when an I2C transaction is in progress, indicating the master is communicating with the slave.
    output reg        no_ack                // A register output that goes high if the slave does not acknowledge (NACK) during communication.
    
    `ifdef DEBUG
    ,
    // =============================================
    // Internal Registers
    // =============================================
    // State machine registers
    output reg [3:0]  current_state,        // Indicates the current state of the FSM.
    output reg [3:0]  future_state,         // Indicates the next state the FSM will transition to.
    
    // I2C control registers
    output reg        enable_scl,           // Enables the 100 kHz I2C clock generator.
    output reg        sda_ctrl,             // Controls the SDA line (drives 0, 1, or high-impedance z).
    // Data and address registers
    output reg [7:0]  device_addr,          // Stores the 7-bit slave address and read/write bit.
    output reg        read_write,           // Stores the read/write bit extracted from dev_address_rw[0].
    output reg [15:0] sec_address,          // Stores the sub-address of the slave.
    output reg        sec_addr_size,        // Stores the sub-address length flag (0 for 8-bit, 1 for 16-bit).
    output reg [7:0]  rx_shift_reg,         // Shift register for receiving data bits from the slave.
    output reg [23:0] bytes_to_transfer,    // Stores the number of bytes to transfer.
    output reg [7:0]  data_buffer,          // Holds the current byte to send or receive.
    
    // Transfer control registers
    output reg        byte_transmitted,     // Flag indicating that a full byte (8 bits) has been sent.
    output reg [23:0] bytes_sent,           // Counts the number of bytes transferred in a transaction.
    output reg [2:0]  bit_counter,          // Counts bits (0 to 7) when transmitting or receiving a byte.
    output reg [7:0]  shift_reg,            // Shift register for sending bits MSB-first during transmission.
    output reg        sec_addr_sent,        // Flag indicating whether the sub-address has been sent.
    
    // 100KHz clock generation
    output reg        i2c_clock,            // The generated 100 kHz I2C clock signal.
    output reg [15:0] clock_counter,        // Counter for dividing the 100 MHz clock to generate the 100 kHz I2C clock.
    
    // Signal sampling registers
    output reg        sda_prev_state,       // Used for sampling and detecting edges on the SDA line.
    output reg [1:0]  sda_current,
    output reg        scl_prev_state,       // Used for sampling and detecting edges on the SCL line.
    output reg        scl_current,
    
    // ACK/NACK control          
    output reg        ack_process,          // Control the acknowledge (ACK) bit logic.
    output reg        ack_status,           
    
    // State machine helpers
    output reg        end_flag,             // Signals that a STOP condition is about to be generated.
    output reg        fetch_data,           // Internal signal to request the next byte for transmission.
    output reg        scl_high,             // Flags to detect SCL edges for timing control.
    output reg        scl_low
    `endif
);
// FSM STATES
localparam [3:0] READY        = 4'd0,       // The READY state, waiting for a new transaction request.
                 INITIATE     = 4'd1,       // Generates the I2C START condition.
                 REINITIATE   = 4'd2,       // Generates a repeated START condition (used for read operations after sending the address).
                 DEV_ADDR     = 4'd3,       // Sends the 7-bit slave address and read/write bit.
                 SEC_ADDR     = 4'd4,       // Sends the sub-address (8-bit or 16-bit).
                 RECEIVE      = 4'd5,       // Receives data bytes from the slave.
                 TRANSMIT     = 4'd6,       // Sends data bytes to the slave.
                 GET_DATA     = 4'd7,       // Requests new data from the external system for transmission.
                 WAIT_ACK     = 4'd8,       // Waits for an acknowledge (ACK) or not-acknowledge (NACK) from the slave.
                 SEND_ACK     = 4'd9,       // Sends an ACK or NACK to the slave during a read operation.
                 TERMINATE    = 4'hA,       // Generates the I2C STOP condition.
                 BUS_RELEASE  = 4'hB;       // Releases the I2C bus and returns to the READY state.

// Change these values according to the speed or board using                 
localparam [15:0] CLK_DIV = 16'd500;       // Divides the 100 MHz input clock to generate a 100 kHz I2C clock. Calculation: $ 100 \, \text{MHz} / 100 \, \text{kHz} = 1000 $ clock cycles per I2C cycle, so $ \text{CLK\_DIV} = 500 $ for a full cycle (high and low phases).
localparam [7:0]  START_SETUP  = 280,      // Number of clock cycles for the setup time of the I2C START condition (SDA goes low while SCL is high).
                  START_HOLD   = 240,      // Number of clock cycles for the hold time of the START condition.
                  DATA_SETUP   =  14,      // Number of clock cycles for the setup time of data bits (SDA stable before SCL rises).
                  DATA_HOLD    =  21,      // Number of clock cycles for the hold time of data bits (SDA stable after SCL rises).
                  STOP_SETUP   = 240;      // Number of clock cycles for the setup time of the STOP condition (SDA goes high while SCL is high).
 /*These values are tuned for the 100 MHz clock and the BASYS-3 FPGA board, ensuring compliance with I2C timing requirements.*/
              
`ifndef DEBUG
reg [3:0]  current_state;
reg [3:0]  future_state;
reg        sda_ctrl;
reg [7:0]  device_addr;
reg        read_write;
reg [15:0] sec_address;
reg        sec_addr_size;
reg [23:0] bytes_to_transfer;
reg        enable_scl;
reg        byte_transmitted;
reg [23:0] bytes_sent;
reg [2:0]  bit_counter;
reg [7:0]  shift_reg;
reg        sec_addr_sent;
reg [7:0]  data_buffer;
reg [7:0]  rx_shift_reg;

reg i2c_clock;
reg [15:0] clock_counter;

reg [1:0] sda_current;
reg       sda_prev_state;
reg scl_prev_state, scl_current;

reg ack_process;
reg ack_status;
reg end_flag;

reg fetch_data;
reg scl_high;
reg scl_low;
`endif
// This block generates the 100 kHz I2C clock (i2c_clock) from the 100 MHz input clock.
// =============================================
// I2C Clock Generation (100kHz from 100MHz)
// =============================================
always@(posedge clock or negedge rst_n) begin
    if(!rst_n)
        {clock_counter, i2c_clock} <= 17'b1;
    else if(!enable_scl)
        {clock_counter, i2c_clock} <= 17'b1;
    else begin
        clock_counter <= clock_counter + 1;
        if(clock_counter == CLK_DIV-1) begin
            i2c_clock <= !i2c_clock;
            clock_counter <= 0;
        end
    end
end
// Main FSM The FSM is the core of the I2C controller, managing the protocol's states and transitions. It's implemented in a single always block sensitive to the positive edge of the clock and the negative edge of the reset. I'll break it down state by state.
/*
 Explanation (Reset Condition):
 ->If rst_n is low (active reset), all registers are initialized:
    rx_data, data_valid: Cleared to 0 (no valid data).
    request_data, active, no_ack: Cleared to 0 (no transaction, no data request, no NACK).
    device_addr, read_write, sec_address, sec_addr_size, bytes_to_transfer, enable_scl: Cleared to 0 (reset address, size, and clock enable).
    byte_transmitted, bytes_sent, bit_counter, shift_reg: Cleared to 0 (reset transmission counters and shift register).
    sec_addr_sent, data_buffer, rx_shift_reg: Cleared to 0 (no sub-address sent, no buffered data).
    ack_status, ack_process, end_flag: Cleared to 0 (reset acknowledge and STOP condition flags).
    scl_high, scl_low, fetch_data: Cleared to 0 (reset timing and data fetch flags).
    sda_ctrl <= 1'bz: Sets the SDA line to high-impedance (idle state).
    current_state <= READY: Sets the FSM to the READY state.
     future_state <= READY: Initializes the next state to READY.
 */ 
 // =============================================
 // Main State Machine
 // =============================================
always@(posedge clock or negedge rst_n) begin
    if(!rst_n) begin
        {rx_data, data_valid} <= 0;
        {request_data, active, no_ack} <= 0;
        {device_addr, read_write, sec_address, sec_addr_size, bytes_to_transfer, enable_scl} <= 0;
        {byte_transmitted, bytes_sent, bit_counter, shift_reg} <= 0;
        {sec_addr_sent, data_buffer, rx_shift_reg} <= 0;
        {ack_status, ack_process, end_flag} <= 0;
        {scl_high, scl_low, fetch_data} <= 0;
        sda_ctrl <= 1'bz;
        current_state <= READY;
        future_state <= READY;
    end
    else begin
        data_valid <= 1'b0;
        request_data <= 1'b0;
        case(current_state)
 /*
  Explanation: READY STATE.
 Global Actions:
     data_valid <= 1'b0: Ensures data_valid is low by default (no valid data unless explicitly set).
     request_data <= 1'b0: Clears the data request signal unless explicitly set.
 --> READY State:
        This is the idle state, where the controller waits for a new transaction.
        if(transfer_request & !active): Checks if a new transaction is requested (transfer_request is high) and no transaction is currently active (!active).
     Actions on Transaction Request:
         active <= 1'b1: Sets the active flag to indicate a transaction is in progress.
         current_state <= INITIATE: Transitions to the INITIATE state to generate the START condition.
         future_state <= DEV_ADDR: Sets the next state to DEV_ADDR (to send the slave address).
         device_addr <= dev_address_rw: Stores the input slave address and read/write bit.
         read_write <= dev_address_rw[0]: Extracts the read/write bit (LSB of dev_address_rw).
         sec_address <= sec_addr_size ? secondary_addr : {secondary_addr[7:0], 8'b0}: Stores the sub-address. If sec_addr_size is 1 (16-bit), the full secondary_addr is used; if 0 (8-bit), the lower 8 bits are used, and the upper 8 bits are padded with zeros.
         sec_addr_size <= sec_addr_len: Stores the sub-address length flag.
         data_buffer <= tx_data: Stores the input data byte for transmission.
         bytes_to_transfer <= data_length: Stores the number of bytes to transfer.
         enable_scl <= 1'b1: Enables the 100 kHz I2C clock.
         sda_ctrl <= 1'b1: Sets the SDA line high (required for the START condition).
         no_ack <= 1'b0: Clears the NACK flag.
         sec_addr_sent <= 1'b0: Indicates the sub-address has not been sent yet.
         bytes_sent <= 0: Resets the count of bytes sent.
         byte_transmitted <= 1'b0: Clears the flag indicating a byte has been transmitted.
 */
            READY: begin
                if(transfer_request & !active) begin
                    active <= 1'b1;
                    current_state <= INITIATE;
                    future_state <= DEV_ADDR;
                    
                    device_addr <= dev_address_rw;
                    read_write <= dev_address_rw[0];
                    sec_address <= sec_addr_size ? secondary_addr : {secondary_addr[7:0], 8'b0};
                    sec_addr_size <= sec_addr_len;
                    data_buffer <= tx_data;
                    bytes_to_transfer <= data_length;

                    enable_scl <= 1'b1;
                    sda_ctrl <= 1'b1;
                    
                    no_ack <= 1'b0;  
                    sec_addr_sent <= 1'b0;
                    bytes_sent <= 0;
                    byte_transmitted <= 1'b0;
                end
            end
 /*
    Explanation: STATE INITIATE
     The INITIATE state checks if the previous state of the SCL (Serial Clock) line (scl_prev_state) and the current SCL (scl_current)
      meet the timing condition (clock_counter == START_SETUP), indicating the start condition for I2C communication.
       When triggered, the FSM sets the SDA (Serial Data) line low (sda_ctrl <= 1'b0), 
       loads the 7-bit device address with a write bit ({device_addr[7:1], 1'b0}) into the shift register, 
       and transitions to the DEV_ADDR state. This marks the beginning of an I2C transaction, where the controller prepares to send the slave device address for data transmission.
 */ 
            INITIATE: begin
                if(scl_prev_state & scl_current & clock_counter == START_SETUP) begin
                    sda_ctrl <= 1'b0;
                    shift_reg <= {device_addr[7:1], 1'b0};
                    current_state <= DEV_ADDR;
                end
            end
 /*
    Explanation: REINTIATE
    This state generates a repeated START condition, typically used for read operations after sending the sub-address.
    if(!scl_current & scl_prev_state): Detects a falling edge of SCL (SCL goes from high to low).
    sda_ctrl <= 1'b1: Sets SDA high to prepare for the repeated START condition.
    if(!scl_prev_state & scl_current): Detects a rising edge of SCL (SCL goes from low to high).
    scl_high <= 1'b1: Sets the scl_high flag to indicate SCL is high.
    if(scl_high): When SCL is high:
    if(clock_counter == START_SETUP): Waits for the START_SETUP time (280 cycles).
    scl_high <= 1'b0: Clears the scl_high flag.
    sda_ctrl <= 1'b0: Pulls SDA low to generate the repeated START condition.
    current_state <= DEV_ADDR: Transitions to the DEV_ADDR state to resend the slave address.
    shift_reg <= device_addr: Loads the full device_addr (including the read/write bit) into the shift register.
 */           
            REINITIATE: begin
                if(!scl_current & scl_prev_state) begin
                    sda_ctrl <= 1'b1;
                end
                
                if(!scl_prev_state & scl_current) begin
                    scl_high <= 1'b1;
                end
                
                if(scl_high) begin
                    if(clock_counter == START_SETUP) begin
                        scl_high <= 1'b0;
                        sda_ctrl <= 1'b0;
                        current_state <= DEV_ADDR;
                        shift_reg <= device_addr;
                    end
                end
            end
 /*
   Explanation: DEV_ADDR.
    The DEV_ADDR state is responsible for transmitting the 7-bit slave address and the read/write bit (8 bits total) to the I2C slave.
    Condition: if(byte_transmitted & bit_counter[0]):
    byte_transmitted: A flag set when all 8 bits of a byte have been transmitted.
    bit_counter[0]: The least significant bit of the 3-bit bit_counter. Since bit_counter counts from 0 to 7, bit_counter[0] is 1 when the counter is odd (e.g., 1, 3, 5, 7). This condition ensures the byte transmission is complete (after 8 bits, bit_counter is 7, and bit_counter[0] is 1).
    When both are true, it means the full 8-bit address (7-bit address + R/W bit) has been sent.
 Actions:
    byte_transmitted <= 1'b0: Clears the byte_transmitted flag to prepare for the next byte.
    future_state <= sec_addr_sent ? RECEIVE : SEC_ADDR: Sets the next state. If the sub-address has already been sent (sec_addr_sent is 1), transition to RECEIVE (for read operations); otherwise, go to SEC_ADDR to send the sub-address.
    shift_reg <= sec_address[15:8]: Loads the upper 8 bits of the sub-address into the shift register, preparing for the SEC_ADDR state (if needed).
    current_state <= WAIT_ACK: Transitions to the WAIT_ACK state to wait for the slave's acknowledge (ACK) or not-acknowledge (NACK).
    sda_ctrl <= 1'bz: Releases the SDA line (sets it to high-impedance) to allow the slave to drive it for the ACK bit.
    bit_counter <= 0: Resets the bit counter for the next byte transmission.
 Else Block: Transmitting Bits:
    if(!scl_current & scl_prev_state): Detects a falling edge of the SCL clock (SCL goes from high to low).
    scl_low <= 1'b1: Sets the scl_low flag to indicate SCL is low, used for timing control.
    if(scl_low): When SCL is low:
    if(clock_counter == DATA_HOLD): Waits for the DATA_HOLD time (21 cycles) to ensure proper data hold timing per the I2C specification.
    scl_low <= 1'b0: Clears the scl_low flag.
    {byte_transmitted, bit_counter} <= {byte_transmitted, bit_counter} + 1: Increments the bit_counter. If bit_counter reaches 7, byte_transmitted is set to 1 (indicating a full byte is sent). The concatenation allows simultaneous increment of both signals.
    sda_ctrl <= shift_reg[7]: Drives the SDA line with the most significant bit (MSB) of the shift_reg (the current bit to send).
    shift_reg <= {shift_reg[6:0], 1'b0}: Shifts the shift_reg left by one bit, moving the next bit to the MSB position and appending a 0 to the LSB.
 */           
 
            DEV_ADDR: begin
                if(byte_transmitted & bit_counter[0]) begin
                    byte_transmitted <= 1'b0;
                    future_state <= sec_addr_sent ? RECEIVE : SEC_ADDR;
                    shift_reg <= sec_address[15:8];
                    current_state <= WAIT_ACK;
                    sda_ctrl <= 1'bz;
                    bit_counter <= 0;
                end
                else begin
                    if(!scl_current & scl_prev_state) begin
                        scl_low <= 1'b1;
                    end
                    
                    if(scl_low) begin
                        if(clock_counter == DATA_HOLD) begin
                            {byte_transmitted, bit_counter} <= {byte_transmitted, bit_counter} + 1;
                            sda_ctrl <= shift_reg[7];
                            shift_reg <= {shift_reg[6:0], 1'b0};
                            scl_low <= 1'b0;
                        end
                    end
                end
            end
 /*
 Explanation: SEC_ADDR
    This state sends the sub-address (8-bit or 16-bit) to the I2C slave, specifying the internal register to read from or write to.
 Condition: if(byte_transmitted & bit_counter[0]):
    Same as in DEV_ADDR, this checks if a full byte (8 bits) has been transmitted.
    Sub-Condition: if(sec_addr_size):
    If sec_addr_size is 1, a 16-bit sub-address is being sent, and this is the first byte (upper 8 bits).
    current_state <= WAIT_ACK: Transitions to WAIT_ACK to wait for the slave's ACK.
    future_state <= SEC_ADDR: Sets the next state to SEC_ADDR to send the second byte of the sub-address.
    sec_addr_size <= 1'b0: Clears the sec_addr_size flag, indicating the next byte is the lower 8 bits.
    shift_reg <= sec_address[7:0]: Loads the lower 8 bits of the sub-address into the shift register.
 Else (8-bit sub-address or second byte of 16-bit):
    future_state <= read_write ? REINITIATE : TRANSMIT: If read_write is 1 (read operation), go to REINITIATE to send a repeated START; otherwise, go to TRANSMIT to send data.
    shift_reg <= read_write ? shift_reg : data_buffer: For read operations, keep the current shift_reg (will be updated in REINITIATE); for write operations, load the data_buffer (data to write).
    sec_addr_sent <= 1'b1: Marks that the sub-address has been sent.
 Common Actions:
    bit_counter <= 0: Resets the bit counter.
    byte_transmitted <= 1'b0: Clears the byte transmission flag.
    current_state <= WAIT_ACK: Transitions to WAIT_ACK to wait for the slave's ACK.
    sda_ctrl <= 1'bz: Releases the SDA line for the ACK bit.
 Else Block: Transmitting Bits:
    Identical to the DEV_ADDR state's bit transmission logic:
    Detects SCL falling edge (!scl_current & scl_prev_state) to set scl_low.
    When clock_counter == DATA_HOLD (21 cycles), sends the next bit by driving sda_ctrl with shift_reg[7], shifts the shift_reg left, and increments bit_counter.
 */           
            SEC_ADDR: begin
                if(byte_transmitted & bit_counter[0]) begin
                    if(sec_addr_size) begin
                        current_state <= WAIT_ACK;
                        future_state <= SEC_ADDR;
                        sec_addr_size <= 1'b0;
                        shift_reg <= sec_address[7:0];
                    end
                    else begin
                        future_state <= read_write ? REINITIATE : TRANSMIT;
                        shift_reg <= read_write ? shift_reg : data_buffer;
                        sec_addr_sent <= 1'b1;
                    end
                    
                    bit_counter <= 0;
                    byte_transmitted <= 1'b0;
                    current_state <= WAIT_ACK;
                    sda_ctrl <= 1'bz;
                end
                else begin
                    if(!scl_current & scl_prev_state) begin
                        scl_low <= 1'b1;
                    end
                    
                    if(scl_low) begin
                        if(clock_counter == DATA_HOLD) begin
                            scl_low <= 1'b0;
                            {byte_transmitted, bit_counter} <= {byte_transmitted, bit_counter} + 1;
                            sda_ctrl <=  shift_reg[7];
                            shift_reg <= {shift_reg[6:0], 1'b0};
                        end
                    end
                end
            end
/*
Explanation:
    This state receives data bytes from the slave during a read operation.
Condition: if(byte_transmitted):
    When a full byte (8 bits) has been received (byte_transmitted is set).
    byte_transmitted <= 1'b0: Clears the flag for the next byte.
    rx_data <= rx_shift_reg: Stores the received byte from rx_shift_reg into rx_data.
    data_valid <= 1'b1: Signals that the data in rx_data is valid.
    current_state <= SEND_ACK: Transitions to SEND_ACK to send an ACK or NACK to the slave.
    future_state <= (bytes_sent == bytes_to_transfer-1) ? TERMINATE : RECEIVE: If all bytes are received (bytes_sent == bytes_to_transfer-1), go to TERMINATE; otherwise, continue receiving in RECEIVE.
    ack_status <= bytes_sent == bytes_to_transfer-1: Sets ack_status to 1 (NACK) for the last byte, 0 (ACK) otherwise.
    bytes_sent <= bytes_sent + 1: Increments the count of received bytes.
    ack_process <= 1'b1: Initiates the ACK process.
Else Block: Receiving Bits:
    if(!scl_prev_state & scl_current): Detects a rising edge of SCL (SCL goes from low to high).
    scl_high <= 1'b1: Sets the scl_high flag.
if(scl_high):
    if(clock_counter == START_SETUP): Waits for the START_SETUP time (280 cycles) to sample SDA.
        data_valid <= 1'b0: Ensures data_valid is low while receiving.
        {byte_transmitted, bit_counter} <= bit_counter + 1: Increments the bit counter, setting byte_transmitted when 8 bits are received.
        rx_shift_reg <= {rx_shift_reg[6:0], sda_prev_state}: Shifts the received bit (sda_prev_state) into the LSB of rx_shift_reg, shifting existing bits left.
        scl_high <= 1'b0: Clears the scl_high flag.
*/            
            RECEIVE: begin
                if(byte_transmitted) begin
                    byte_transmitted <= 1'b0;
                    rx_data  <= rx_shift_reg;
                    data_valid <= 1'b1;
                    current_state <= SEND_ACK;
                    future_state <= (bytes_sent == bytes_to_transfer-1) ? TERMINATE : RECEIVE;
                    ack_status <= bytes_sent == bytes_to_transfer-1;
                    bytes_sent <= bytes_sent + 1;
                    ack_process <= 1'b1;
                end
                else begin
                    if(!scl_prev_state & scl_current) begin
                        scl_high <= 1'b1;
                    end
                    
                    if(scl_high) begin
                        if(clock_counter == START_SETUP) begin
                            data_valid <= 1'b0;
                            {byte_transmitted, bit_counter} <= bit_counter + 1;
                            rx_shift_reg <= {rx_shift_reg[6:0], sda_prev_state};
                            scl_high <= 1'b0;
                        end
                    end
                end
            end
/*
    Explanation:
        This state transmits data bytes to the slave during a write operation.
    Condition: if(byte_transmitted & bit_counter[0]):
            When a full byte is transmitted.
            bit_counter <= 0: Resets the bit counter.
            byte_transmitted <= 1'b0: Clears the byte transmission flag.
            current_state <= WAIT_ACK: Transitions to WAIT_ACK to wait for the slave's ACK.
            sda_ctrl <= 1'bz: Releases the SDA line for the ACK bit.
            future_state <= (bytes_sent == bytes_to_transfer-1) ? TERMINATE : GET_DATA: If all bytes are sent, go to TERMINATE; otherwise, go to GET_DATA to fetch the next byte.
            bytes_sent <= bytes_sent + 1'b1: Increments the count of sent bytes.
            fetch_data <= 1'b1: Signals to fetch the next data byte.
    Else Block: Transmitting Bits:
            Identical to the DEV_ADDR and SEC_ADDR bit transmission logic:
            Detects SCL falling edge to set scl_low.
            At DATA_HOLD time, sends the next bit, shifts shift_reg, and increments bit_counter.
*/            
            TRANSMIT: begin
                if(byte_transmitted & bit_counter[0]) begin
                    bit_counter <= 0;
                    byte_transmitted <= 1'b0;
                    current_state <= WAIT_ACK;
                    sda_ctrl <= 1'bz;
                    future_state <= (bytes_sent == bytes_to_transfer-1) ? TERMINATE : GET_DATA;
                    bytes_sent <= bytes_sent + 1'b1;
                    fetch_data <= 1'b1;
                end
                else begin
                    if(!scl_current & scl_prev_state) begin
                        scl_low <= 1'b1;
                    end
                    
                    if(scl_low) begin
                        if(clock_counter == DATA_HOLD) begin
                            {byte_transmitted, bit_counter} <= {byte_transmitted, bit_counter} + 1;
                            sda_ctrl <= shift_reg[7];
                            shift_reg <= {shift_reg[6:0], 1'b0};
                            scl_low <= 1'b0;
                        end
                    end
                end
            end
/*Explanation:
            This state requests new data from the external system for transmission.*/            
            GET_DATA: begin
                if(fetch_data) begin  // Signals the external system to provide new data in tx_data.
                    request_data <= 1'b1;
                    fetch_data <= 1'b0;  // Clears the fetch_data flag.
                end
                else begin
                    current_state <= TRANSMIT; // Transitions back to TRANSMIT to send the new data.
                    shift_reg <= tx_data;      // Loads the new data byte from tx_data into the shift register.
                end
            end
/*
Explanation:
    This state waits for the slave's ACK (SDA low) or NACK (SDA high) after sending a byte.
    if(!scl_prev_state & scl_current): Detects a rising edge of SCL.
        scl_high <= 1'b1: Sets the scl_high flag.
    if(scl_high):
        if(clock_counter == START_SETUP): Samples SDA at the START_SETUP time.
            if(!sda_prev_state): If SDA is low (ACK), proceed to the future_state (e.g., SEC_ADDR, RECEIVE, or TRANSMIT).
        Else (NACK):
            no_ack <= 1'b1: Sets the NACK flag to indicate the slave did not acknowledge.
            active <= 1'b0: Clears the active transaction flag.
            sda_ctrl <= 1'bz: Releases the SDA line.
            enable_scl <= 1'b0: Disables the I2C clock.
            current_state <= READY: Returns to the idle state.
            scl_high <= 1'b0: Clears the scl_high flag.
*/            
            WAIT_ACK: begin
                if(!scl_prev_state & scl_current) begin
                    scl_high <= 1'b1;
                end
                
                if(scl_high) begin
                    if(clock_counter == START_SETUP) begin
                        if(!sda_prev_state) begin
                            current_state <= future_state;
                        end
                        else begin
                            no_ack <= 1'b1;
                            active <= 1'b0;
                            sda_ctrl <= 1'bz;
                            enable_scl <= 1'b0;
                            current_state <= READY;
                        end  
                        scl_high <= 1'b0;
                    end
                end
            end
/*
   Explanation:
    This state sends an ACK (SDA low) or NACK (SDA high) to the slave during a read operation.
    if(!scl_current & scl_prev_state): Detects a falling edge of SCL.
        scl_low <= 1'b1: Sets the scl_low flag.
    if(scl_low):
        if(clock_counter == DATA_HOLD):
            if(ack_process): If in the ACK process:
                sda_ctrl <= ack_status: Drives SDA with ack_status (0 for ACK, 1 for NACK on the last byte).
                ack_process <= 1'b0: Clears the ack_process flag.
            Else:
                sda_ctrl <= future_state == TERMINATE ? 1'b0 : 1'bz: If transitioning to TERMINATE, sets SDA low to prepare for the STOP condition; otherwise, releases SDA.
                end_flag <= future_state == TERMINATE ? 1'b1 : end_flag: Sets the end_flag if terminating.
                current_state <= future_state: Moves to the next state (RECEIVE or TERMINATE).
s               cl_low <= 1'b0: Clears the scl_low flag.
*/            
            SEND_ACK: begin
                if(!scl_current & scl_prev_state) begin
                    scl_low <= 1'b1;
                end
                if(scl_low) begin
                    if(clock_counter == DATA_HOLD) begin
                        if(ack_process) begin 
                            sda_ctrl <= ack_status;
                            ack_process <= 1'b0;
                        end
                        else begin
                            sda_ctrl <= future_state == TERMINATE ? 1'b0 : 1'bz;
                            end_flag <= future_state == TERMINATE ? 1'b1 : end_flag;
                            current_state <= future_state;
                        end
                        scl_low <= 1'b0;
                    end
                end
            end
 /*
    Explanation:
    This state generates the I2C STOP condition (SDA goes from low to high while SCL is high).
    if(!scl_current & scl_prev_state & !read_write): On SCL falling edge and for write operations:
        sda_ctrl <= 1'b0: Sets SDA low to prepare for the STOP condition.
        end_flag <= 1'b1: Sets the end_flag.
    if(scl_current & scl_prev_state & end_flag): When SCL is high and end_flag is set:
        scl_high <= 1'b1: Sets the scl_high flag.
        end_flag <= 1'b0: Clears the end_flag.
    if(scl_high):
        if(clock_counter == STOP_SETUP): At STOP_SETUP time (240 cycles):
            sda_ctrl <= 1'b1: Sets SDA high to complete the STOP condition.
            current_state <= BUS_RELEASE: Transitions to BUS_RELEASE to release the bus.
            scl_high <= 1'b0: Clears the scl_high flag.
 */           
            TERMINATE: begin 
                if(!scl_current & scl_prev_state & !read_write) begin
                    sda_ctrl <= 1'b0;
                    end_flag <= 1'b1;
                end
                
                if(scl_current & scl_prev_state & end_flag) begin
                    scl_high <= 1'b1;
                    end_flag <= 1'b0;
                end
                
                if(scl_high) begin
                    if(clock_counter == STOP_SETUP) begin
                        sda_ctrl <= 1'b1;
                        current_state <= BUS_RELEASE;
                        scl_high <= 1'b0;
                    end
                end
            end
 /*
   Explanation:
    This state releases the I2C bus and returns to the idle state.
    if(clock_counter == CLK_DIV-3): Waits for a short delay (497 cycles) to ensure proper bus release.
        enable_scl <= 1'b0: Disables the I2C clock.
        current_state <= READY: Returns to the READY state.
        sda_ctrl <= 1'bz: Releases the SDA line.
        active <= 1'b0: Clears the active transaction flag.
 */           
            BUS_RELEASE: begin
                if(clock_counter == CLK_DIV-3) begin
                    enable_scl <= 1'b0;
                    current_state <= READY;
                    sda_ctrl <= 1'bz;
                    active <= 1'b0;
                end
            end
 /*
 Explanation:
    assign sda_line = sda_ctrl: Directly connects the sda_ctrl register to the sda_line output. sda_ctrl can be 0, 1, or z (high-impedance), controlling the SDA line's state.
    assign scl_line = enable_scl ? i2c_clock : 1'bz: Drives the SCL line with the 100 kHz i2c_clock when enable_scl is high; otherwise, sets SCL to high-impedance (1'bz) to release the bus.
 */           
            default:
                current_state <= READY;
        endcase
    end
end
/*
Explanation:
    This block samples the SDA and SCL lines on the negative edge of the 100 MHz clock to detect edges and synchronize signals.
    Reset Condition:
    if(!rst_n): On reset, clears sda_current, sda_prev_state, scl_current, and scl_prev_state to 0.
Sampling:
    sda_current <= {sda_current[0], sda_line}: Shifts the current SDA value into a 2-bit shift register (sda_current[0] becomes the previous value, sda_line becomes the new value).
    sda_prev_state <= sda_current[1]: Stores the previous SDA value for edge detection.
    scl_current <= i2c_clock: Samples the current I2C clock value.
    scl_prev_state <= scl_current: Stores the previous SCL value for edge detection.
    This allows the FSM to detect rising (!scl_prev_state & scl_current) and falling (!scl_current & scl_prev_state) edges of SCL, and to sample SDA for data and ACK bits.
*/
// =============================================
// SDA/SCL Input Synchronization
// =============================================
always@(negedge clock or negedge rst_n) begin
    if(!rst_n) begin
        {sda_current, sda_prev_state} <= 0;
        {scl_current, scl_prev_state} <= 0;
    end
    else begin
        sda_current <= {sda_current[0], sda_line};
        sda_prev_state <= sda_current[1];
        scl_current <= i2c_clock;
        scl_prev_state <= scl_current;
    end
end
/*Explanation:
    assign sda_line = sda_ctrl: Directly connects the sda_ctrl register to the sda_line output. sda_ctrl can be 0, 1, or z (high-impedance), controlling the SDA line's state.
    assign scl_line = enable_scl ? i2c_clock : 1'bz: Drives the SCL line with the 100 kHz i2c_clock when enable_scl is high; otherwise, sets SCL to high-impedance (1'bz) to release the bus.*/
// =============================================
// I2C Line Drivers
// =============================================
assign sda_line = sda_ctrl;
assign scl_line = enable_scl ? i2c_clock : 1'bz;
endmodule
