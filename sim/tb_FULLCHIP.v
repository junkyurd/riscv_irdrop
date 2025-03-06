`timescale 1ns / 1ps

module counter_16bit (
    input wire clk,        // Clock input
    input wire reset,      // Synchronous reset input
    input wire enable,     // Enable input
    output wire [15:0] out  // 16-bit output
);

    reg [15:0] counter_stage;
    assign out[15:0] = counter_stage[15:0];
    // Counter logic
    always @(posedge clk) begin
        if (reset) begin
            // When reset is high, set the counter to 0
            counter_stage <= 16'b0;
        end else if (enable) begin
            // Increment the counter if enable is high
            counter_stage <= counter_stage + 1'b1;
        end
        // If enable is low, do nothing (hold value)
    end

endmodule

module tb_FULLCHIP;

    reg clk;
    reg resetb;

    reg A_WRITE_READ_SEL;
    reg [5:0] AADDR;
    reg AVALID;
    wire AREADY;
    //reg [5:0] AWADDR;
    //reg AWVALID;
    //wire AWREADY;
    reg [7:0] WDATA;
    reg WVALID;
    wire WREADY;
    //reg [5:0] ARADDR;
    //reg ARVALID;
    //wire ARREADY;
    wire [7:0] RDATA;
    wire RVALID;
    reg RREADY;

    reg HEATER_PWM_A;
    reg HEATER_PWM_B;
    wire SENSE_HI;
    wire SENSE_LO;
    wire SOURCE_HI;
    wire SOURCE_LO;

    wire FREQ_OUT;
    wire DEBUG_OUT;

    wire [15:0] counter_out;
    reg counter_en;

    reg [31:0] instr_data[300:0];

    wire resetb_delayed;
    wire A_WRITE_READ_SEL_delayed;
    wire [5:0] AADDR_delayed;
    wire AVALID_delayed;
    wire AREADY_delayed;
    wire [7:0] WDATA_delayed;
    wire WVALID_delayed;
    wire WREADY_delayed;
    wire [7:0] RDATA_delayed;
    wire RVALID_delayed;
    wire RREADY_delayed;
    wire HEATER_PWM_A_delayed;
    wire HEATER_PWM_B_delayed;


    //Power
    wire VDD0P9_TOP, VDD0P9_CORE1, VDD0P9_CORE2, VSS0P9;

    parameter clk_period = 50;//20MHz clock
    parameter inst_line_number = 26;//Put the length of instruction file here.
    always begin
        clk = ~clk;
        #(clk_period/2);
    end

    always begin
        HEATER_PWM_A = ~HEATER_PWM_A;
        HEATER_PWM_B = ~HEATER_PWM_B;
        #30;
    end

    assign VDD0P9_TOP      = 1'b1;
    assign VDD0P9_CORE1    = 1'b1;
    assign VDD0P9_CORE2    = 1'b1;
    assign VSS0P9      = 1'b0;

    localparam
        ADDR_INSTR_ADDR                 = 6'h00,
        ADDR_INSTR_DATA_IN              = 6'h01,
        ADDR_INSTR_DATA_OUT             = 6'h02, // READ ONLY
        ADDR_DATA_ADDR                  = 6'h03,
        ADDR_DATA_DATA_IN               = 6'h04,
        ADDR_DATA_DATA_OUT              = 6'h05, // READ ONLY
        ADDR_CACHE_CONTROL              = 6'h06; //[0] Instr read start [1] Data read start

    // Miscellaneous
    localparam
        ADDR_DELAY_SEL                  = 6'h0E;
    
    localparam
        ADDR_DDLS_OFFSET                = 6'h0F,
        ADDR_DDLS_ADDR                  = 6'h10,
        ADDR_DDLS_DATA_IN               = 6'h11,
        ADDR_DDLS_DATA_OUT              = 6'h12;

    localparam
        ADDR_CLK_CONTROL                = 6'h13,
        ADDR_MAIN_DCO_CONTROL           = 6'h14, // [7:4] COARSE [3:0] FINE
        ADDR_RISCV_RUN_CYCLE_OFFSET     = 6'h15;

    // RISC-V control
    localparam
        ADDR_BOOT_ADDR                  = 6'h07,
        ADDR_MTVEC_ADDR                 = 6'h08,
        ADDR_RISCV_START                = 6'h09,
        ADDR_RISCV_OFFSET               = 6'h0A,
        ADDR_RISCV_RUN_CYCLE            = 6'h0B,
        ADDR_RISCV_STATUS               = 6'h0C,
        ADDR_RISCV_ERRCNT               = 6'h0D; // Read only

    // Odometer
    localparam
        ADDR_ODOMETER_SEL_7_0           = 6'h29,
        ADDR_ODOMETER_SEL_15_8          = 6'h2A,
        ADDR_ODOMETER_IN                = 6'h2B,
        ADDR_ODOMETER_CONTROL           = 6'h2C,
        ADDR_ODOMETER_BIT_CNT_7_0       = 6'h2D,
        ADDR_ODOMETER_BIT_CNT_15_8      = 6'h2E;

    // Heater control
    localparam
        ADDR_HEATER_TOP_SEL             = 6'h20,
        ADDR_HEATER_TOP_IN              = 6'h21;
    localparam
        ADDR_HEATER_P_SEL               = 6'h22,
        ADDR_HEATER_P_IN                = 6'h23;
    localparam
        ADDR_HEATER_S_SEL               = 6'h24,
        ADDR_HEATER_S_IN                = 6'h25;
    localparam
        ADDR_HEATER_CONTROL             = 6'h26; // {GLOBAL_ENABLE, 4'b0000, LOAD_HEATER_TOP, LOAD_HEATER_P, LOAD_HEATER_S}
    localparam
        ADDR_TEMP_SENSOR_SEL            = 6'h27,
        ADDR_VOLTAGE_SENSOR_SEL         = 6'h28;
    localparam
        ADDR_DEBUG_FREQ_OUT_SEL         = 6'h30;
    localparam
        ADDR_MAIN_DCO_EN                = 6'h3F;

    initial begin
        clk = 1'b0;
        resetb = 1'b0;
        A_WRITE_READ_SEL = 1'b0;
        AADDR = 1'b0;
        AVALID = 1'b0;
        WDATA = 0;
        WVALID = 1'b0;
        RREADY = 1'b0;
        HEATER_PWM_A = 1'b0;
        HEATER_PWM_B = 1'b1;
        #100;
        resetb = 1'b1;
        RREADY = 1'b1;
        
        
        run_delayed_ddls;
        /////////////////////////////////////////////////////////////////
        // ODOMETER
        // odometer_test;
        /////////////////////////////////////////////////////////////////


        /////////////////////////////////////////////////////////////////
        //Full stack operation: Odometer
        //fs_odometer;
        /////////////////////////////////////////////////////////////////

        /////////////////////////////////////////////////////////////////
        // HEATER
        // heater_test;
        /////////////////////////////////////////////////////////////////
        
        
        /////////////////////////////////////////////////////////////////
        //Cache
        // instruction_write_read;
        // data_write_read;
        /////////////////////////////////////////////////////////////////

        /////////////////////////////////////////////////////////////////
        //Run DDLS
        // runddls;
        // ddls_write_read;
        /////////////////////////////////////////////////////////////////

        /////////////////////////////////////////////////////////////////
        //CPU simple operation and READ Data cache
        // runcpu;
        // runcpu_pulse;
        /////////////////////////////////////////////////////////////////

        /////////////////////////////////////////////////////////////////
        //RISCV_DCO programming 
        // dco_test;
        // dco_count_test;
        // odo_dco_test;
        /////////////////////////////////////////////////////////////////

        /////////////////////////////////////////////////////////////////
        //Full stack operation: RUN CPU
        //fs_cpu;
        /////////////////////////////////////////////////////////////////


        #(clk_period*10);
        
        $finish;
    end

// --------------------------------------------------------------------
// 0   0  000  0000  0   0 0     00000
// 00 00 0   0 0   0 0   0 0     0
// 0 0 0 0   0 0   0 0   0 0     00000
// 0   0 0   0 0   0 0   0 0     0
// 0   0  000  0000   000  00000 00000
// --------------------------------------------------------------------
    // assign #5.5 resetb_delayed = resetb;
    // assign #5.5 A_WRITE_READ_SEL_delayed = A_WRITE_READ_SEL;
    // assign #5.5 AADDR_delayed = AADDR;
    // assign #5.5 AVALID_delayed = AVALID;
    // assign #5.5 AREADY_delayed = AREADY;
    // assign #5.5 WDATA_delayed = WDATA;
    // assign #5.5 WVALID_delayed = WVALID;
    // assign #5.5 WREADY_delayed = WREADY;
    // assign #5.5 RDATA_delayed = RDATA;
    // assign #5.5 RVALID_delayed = RVALID;
    // assign #5.5 RREADY_delayed = RREADY;
    // assign #5.5 HEATER_PWM_A_delayed = HEATER_PWM_A;
    // assign #5.5 HEATER_PWM_B_delayed = HEATER_PWM_B;

    FULLCHIP # (
        .NUM_ODOMETER_P(21),
        .NUM_ODOMETER_S(21),
        .NUM_HEATER_P(84),
        .NUM_HEATER_S(84) 
    ) FULLCHIP_inst (
        .ACLK(clk),
        .ARESETB(resetb),
        .AWRITEB(A_WRITE_READ_SEL),
        .AADDR(AADDR),
        .AVALID(AVALID),
        .WDATA(WDATA),
        .WVALID(WVALID),
        .RREADY(RREADY),
        .HEATER_PWM_A(HEATER_PWM_A),
        .HEATER_PWM_B(HEATER_PWM_B),
        .AREADY(AREADY),
        .WREADY(WREADY),
        .RDATA(RDATA),
        .RVALID(RVALID),
        .DEBUG_OUT(DEBUG_OUT),
        .DCO_FREQ_OUT(FREQ_OUT),
        .TEMP_SOURCE_HI(SOURCE_HI),
        .TEMP_SOURCE_LO(SOURCE_LO),
        .TEMP_SENSE_HI(SENSE_HI),
        .TEMP_SENSE_LO(SOURCE_LO),
        .VDD0P9_TOP(VDD0P9_TOP),
        .VDD0P9_CORE1(VDD0P9_CORE1),
        .VDD0P9_CORE2(VDD0P9_CORE2),
        .VSS0P9(VSS0P9),
        .VDD1P8(VDD1P8),
        .VSS1P8(VSS1P8)
    );

    counter_16bit freq_counter (.clk(clk), .reset(~resetb), .out(counter_out), .enable(counter_en));

    initial begin 
		$dumpfile("dump.vcd"); $dumpvars;
	end

// --------------------------------------------------------------------
// 00000   0    000  0   0
//   0    0 0  0     0  0
//   0   00000  000  000
//   0   0   0     0 0  0
//   0   0   0  000  0   0
// --------------------------------------------------------------------

    task write_stream (input [6:0] addr, input [7:0] data);
        begin
            @(posedge clk);
            A_WRITE_READ_SEL <= 1'b0;
            @(posedge clk);
            AADDR <= addr;
            WDATA <= data;
            AVALID <= 1'b1;
            @(posedge clk);
            while (!(AREADY)) begin
                @(posedge clk);
            end
            
            AVALID <= 1'b0;
            WVALID <= 1'b1;
            @(posedge clk);
            while (!(WREADY)) begin
                @(posedge clk);
            end
            WVALID <= 1'b0;
        end
    endtask

    task read_stream (input [6:0] addr);
        begin
            @(posedge clk);
            A_WRITE_READ_SEL <= 1'b1;
            @(posedge clk);
            AADDR <= addr;            
            AVALID <= 1'b1;
            @(posedge clk);
            while (!(AREADY)) begin
                @(posedge clk);
            end            
            AVALID <= 1'b0;
            while(!(RVALID)) begin
                @(posedge clk);
            end
        end
    endtask

    task instruction_write_read;
        // Reads in instruction memory file
        $readmemh("/project/chriskim00/hanzhao/CRYO_RISCV/RARE_CHIP_POST_LAYOUT_11_07/rtl/inst_h.mem_sim", instr_data);

        //---------------Instraction Cache Write----------------------------//
        write_stream(ADDR_RISCV_OFFSET, 8'b00000000); // OFFSET for writing instruction data is 0, means start writing at [7:0]
        // Set instr_addr to 32'h00000000
        write_stream(ADDR_INSTR_ADDR, 8'h00); // instr_addr[7:0] = 8'h00
        write_stream(ADDR_INSTR_ADDR, 8'h00); // instr_addr[15:8] = 8'h00
        write_stream(ADDR_INSTR_ADDR, 8'h00); // instr_addr[23:16] = 8'h00
        write_stream(ADDR_INSTR_ADDR, 8'h00); // instr_addr[31:24] = 8'h00

        write_stream(ADDR_CACHE_CONTROL, 8'h40); // Stream_sel = 10, writes to primary

        // start writing instruction data
        for (int w = 0; w < inst_line_number; w = w+1) begin
            write_stream(ADDR_INSTR_DATA_IN, instr_data[w][7:0]); // instr_data[7:0]
            write_stream(ADDR_INSTR_DATA_IN, instr_data[w][15:8]); // instr_data[15:8]
            write_stream(ADDR_INSTR_DATA_IN, instr_data[w][23:16]); // instr_data[23:16]
            write_stream(ADDR_INSTR_DATA_IN, instr_data[w][31:24]); // instr_data[31:24]
        end
        // end writing instruction.
        //---------------Instraction Cache Write----------------------------//

        #(clk_period*10);; // wait

        //---------------Instraction Cache Read----------------------------//
        // read back written instruction
        write_stream(ADDR_RISCV_OFFSET, 8'b11110100);
        // Set instr_addr to 32'h00000000
        write_stream(ADDR_INSTR_ADDR, 8'h00);
        write_stream(ADDR_INSTR_ADDR, 8'h00);
        write_stream(ADDR_INSTR_ADDR, 8'h00);
        write_stream(ADDR_INSTR_ADDR, 8'h00);

        for (int w = 0; w < inst_line_number; w = w+1) begin
            write_stream(ADDR_CACHE_CONTROL, 8'h41);//[7:6]: 01/11: stream from primary, 10: stream from secondary
            
            do begin
                read_stream(ADDR_CACHE_CONTROL);
            end while(RDATA[2] != 1'b1);
            
            read_stream(ADDR_INSTR_DATA_OUT);
            read_stream(ADDR_INSTR_DATA_OUT);
            read_stream(ADDR_INSTR_DATA_OUT);
            read_stream(ADDR_INSTR_DATA_OUT);
        end
        //---------------Instraction Cache Read----------------------------//

        // write_stream(ADDR_RISCV_START, 8'b00001111); // 1111
    endtask

    task ddls_write_read;
        // Reads in instruction memory file
        $readmemh("/project/chriskim00/hanzhao/CRYO_RISCV/RARE_GATE_SIM_10_31/rtl/inst_h.mem_sim", instr_data);

        //---------------DDLS Cache Write----------------------------//
        write_stream(ADDR_DDLS_OFFSET, 8'b00000000);
        
        write_stream(ADDR_DDLS_ADDR, 8'h00);
        write_stream(ADDR_DDLS_ADDR, 8'h00);
        write_stream(ADDR_DDLS_ADDR, 8'h00);
        write_stream(ADDR_DDLS_ADDR, 8'h00);

        for (int w = 0; w < inst_line_number; w = w+1) begin
            write_stream(ADDR_DDLS_DATA_IN, instr_data[w][7:0]); // instr_data[7:0]
            write_stream(ADDR_DDLS_DATA_IN, instr_data[w][15:8]); // instr_data[15:8]
            write_stream(ADDR_DDLS_DATA_IN, instr_data[w][23:16]); // instr_data[23:16]
            write_stream(ADDR_DDLS_DATA_IN, instr_data[w][31:24]); // instr_data[31:24]
        end
        //---------------DDLS Cache Write----------------------------//

        #(clk_period*10);; // wait

        //-----------------Read from DDLS cache------------------------//
        write_stream(ADDR_RISCV_OFFSET, 8'b00000000);
        // Set data_addr to 32'h00000000
        write_stream(ADDR_DDLS_ADDR, 8'h00);
        write_stream(ADDR_DDLS_ADDR, 8'h00);
        write_stream(ADDR_DDLS_ADDR, 8'h00);
        write_stream(ADDR_DDLS_ADDR, 8'h00);

        // To save simulation time, we just read out the first 4 cache address, for real testing, we will need to read "RISCV_ERRCNT" number of address
        for (int w = 0; w < 4; w = w+1) begin
            write_stream(ADDR_DDLS_OFFSET, 8'b00000000);
            write_stream(ADDR_CACHE_CONTROL, 8'h10);
            
            do begin
                read_stream(ADDR_CACHE_CONTROL);
            end while(RDATA[5] != 1'b1);
            
            // We have 256bit, so we will need to read 32 times.
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
        end
    endtask

    task data_write_read;
        //For sake of verification, we just use the same data file as instruction cache and see if it can write.
        $readmemh("/project/chriskim00/hanzhao/CRYO_RISCV/RARE_CHIP_POST_LAYOUT_11_07/rtl/inst_h.mem_sim", instr_data);
        //---------------Data Cache Write----------------------------//
        write_stream(ADDR_RISCV_OFFSET, 8'b00000000);// OFFSET for writing instruction data is 0, means start writing at [7:0]
        // Set instr_addr to 32'h00000000
        write_stream(ADDR_DATA_ADDR, 8'h00);
        write_stream(ADDR_DATA_ADDR, 8'h00);
        write_stream(ADDR_DATA_ADDR, 8'h00);
        write_stream(ADDR_DATA_ADDR, 8'h00);
        
        write_stream(ADDR_CACHE_CONTROL, 8'hC0); // Stream_sel = 11, writes to both primary and secondary
        for (int w = 0; w < inst_line_number; w = w+1) begin
            write_stream(ADDR_DATA_DATA_IN, instr_data[w][7:0]); // instr_data[7:0]
            write_stream(ADDR_DATA_DATA_IN, instr_data[w][15:8]); // instr_data[15:8]
            write_stream(ADDR_DATA_DATA_IN, instr_data[w][23:16]);// instr_data[23:16]
            write_stream(ADDR_DATA_DATA_IN, instr_data[w][31:24]);// instr_data[31:24]
        end
        //---------------Data Cache Write----------------------------//

        #(clk_period*10);; // wait

        //---------------Data Cache Read----------------------------//
        // read back written Data
        write_stream(ADDR_RISCV_OFFSET, 8'b00000000);
        // Set data_addr to 32'h00000000
        write_stream(ADDR_DATA_ADDR, 8'h00);
        write_stream(ADDR_DATA_ADDR, 8'h00);
        write_stream(ADDR_DATA_ADDR, 8'h00);
        write_stream(ADDR_DATA_ADDR, 8'h00);

        for (int w = 0; w < inst_line_number; w = w+1) begin
            write_stream(ADDR_CACHE_CONTROL, 8'h42);
            
            do begin
                read_stream(ADDR_CACHE_CONTROL);
            end while(RDATA[3] != 1'b1);
            
            read_stream(ADDR_DATA_DATA_OUT);
            read_stream(ADDR_DATA_DATA_OUT);
            read_stream(ADDR_DATA_DATA_OUT);
            read_stream(ADDR_DATA_DATA_OUT);
        end
        //---------------Data Cache Read----------------------------//

        #(clk_period*10); // wait  
    endtask

    task runddls;
        // Reads in instruction memory file
        $readmemh("/project/chriskim00/hanzhao/CRYO_RISCV/RARE_CHIP_POST_LAYOUT_11_07/rtl/inst_h.mem_sim_p", instr_data);
        //---------------Instraction Cache Write----------------------------//
        write_stream(ADDR_RISCV_OFFSET, 8'b00000000); // OFFSET for writing instruction data is 0, means start writing at [7:0]
        // Set instr_addr to 32'h00000000
        write_stream(ADDR_INSTR_ADDR, 8'h00); // instr_addr[7:0] = 8'h00
        write_stream(ADDR_INSTR_ADDR, 8'h00); // instr_addr[15:8] = 8'h00
        write_stream(ADDR_INSTR_ADDR, 8'h00); // instr_addr[23:16] = 8'h00
        write_stream(ADDR_INSTR_ADDR, 8'h00); // instr_addr[31:24] = 8'h00

        write_stream(ADDR_CACHE_CONTROL, 8'h40); // Stream_sel = 0100 0000, writes primary $I cache

        // start writing instruction data
        for (int w = 0; w < inst_line_number; w = w+1) begin
            write_stream(ADDR_INSTR_DATA_IN, instr_data[w][7:0]); // instr_data[7:0]
            write_stream(ADDR_INSTR_DATA_IN, instr_data[w][15:8]); // instr_data[15:8]
            write_stream(ADDR_INSTR_DATA_IN, instr_data[w][23:16]); // instr_data[23:16]
            write_stream(ADDR_INSTR_DATA_IN, instr_data[w][31:24]); // instr_data[31:24]
        end


        $readmemh("/project/chriskim00/hanzhao/CRYO_RISCV/RARE_CHIP_POST_LAYOUT_11_07/rtl/inst_h.mem_sim_s", instr_data);
        //---------------Instraction Cache Write----------------------------//
        write_stream(ADDR_RISCV_OFFSET, 8'b00000000); // OFFSET for writing instruction data is 0, means start writing at [7:0]
        // Set instr_addr to 32'h00000000
        write_stream(ADDR_INSTR_ADDR, 8'h00); // instr_addr[7:0] = 8'h00
        write_stream(ADDR_INSTR_ADDR, 8'h00); // instr_addr[15:8] = 8'h00
        write_stream(ADDR_INSTR_ADDR, 8'h00); // instr_addr[23:16] = 8'h00
        write_stream(ADDR_INSTR_ADDR, 8'h00); // instr_addr[31:24] = 8'h00

        write_stream(ADDR_CACHE_CONTROL, 8'h80); // Stream_sel = 1000 0000, writes secondary $I cache

        // start writing instruction data
        for (int w = 0; w < inst_line_number; w = w+1) begin
            write_stream(ADDR_INSTR_DATA_IN, instr_data[w][7:0]); // instr_data[7:0]
            write_stream(ADDR_INSTR_DATA_IN, instr_data[w][15:8]); // instr_data[15:8]
            write_stream(ADDR_INSTR_DATA_IN, instr_data[w][23:16]); // instr_data[23:16]
            write_stream(ADDR_INSTR_DATA_IN, instr_data[w][31:24]); // instr_data[31:24]
        end



        //---------------Enable DCO----------------------------//
        write_stream(ADDR_CLK_CONTROL,8'b01000000);
        write_stream(ADDR_MAIN_DCO_CONTROL, 8'h00); // Write value for FINE and COARSE
        write_stream(ADDR_MAIN_DCO_EN, 8'h01);      // Enable DCO
        //---------------Enable DCO----------------------------//

        write_stream(ADDR_DELAY_SEL, 3); // DDLS secondary delay is 2 cycles
        write_stream(ADDR_RISCV_RUN_CYCLE_OFFSET, 8'b00000000); // OFFSET for writing run cycle is 0, means start writing at [7:0], also enalbe run cycle count
        
        write_stream(ADDR_RISCV_RUN_CYCLE, 8'h00); // will run for 4 cycles (64-1)
        write_stream(ADDR_RISCV_RUN_CYCLE, 8'h00);
        write_stream(ADDR_RISCV_RUN_CYCLE, 8'h00);
        write_stream(ADDR_RISCV_RUN_CYCLE, 8'h10);

        write_stream(ADDR_RISCV_OFFSET, 8'b00000000);

        write_stream(ADDR_BOOT_ADDR, 8'h00); // BOOT ADDR, first address instructio is read is at 0x00
        write_stream(ADDR_BOOT_ADDR, 8'h00);
        write_stream(ADDR_BOOT_ADDR, 8'h00);
        write_stream(ADDR_BOOT_ADDR, 8'h00);

        write_stream(ADDR_MTVEC_ADDR, 8'h00); // MTVEC ADDR, not used for this RISCV
        write_stream(ADDR_MTVEC_ADDR, 8'h00);
        write_stream(ADDR_MTVEC_ADDR, 8'h00);
        write_stream(ADDR_MTVEC_ADDR, 8'h00);

        write_stream(ADDR_RISCV_START, 8'b00000010); // [0] RISCV RESETB [1] FETCH_ENABLE [2] RISCV_CLK_EN [3] DDLS_RESETB
        write_stream(ADDR_RISCV_START, 8'b00001010); // [0] RISCV RESETB [1] FETCH_ENABLE [2] RISCV_CLK_EN [3] DDLS_RESETB
        write_stream(ADDR_RISCV_START, 8'b00001110); // [0] RISCV RESETB [1] FETCH_ENABLE [2] RISCV_CLK_EN [3] DDLS_RESETB

        write_stream(ADDR_RISCV_START, 8'b00001111); // [0] RISCV RESETB [1] FETCH_ENABLE [2] RISCV_CLK_EN [3] DDLS_RESETB

        do begin // wait until done
           read_stream(ADDR_RISCV_STATUS);
        end while(RDATA[0] != 1'b1);     // If we are running two core
        //end while(RDATA[6] != 1'b1);      // If we just run primary
        write_stream(ADDR_RISCV_START, 8'b00001101);//Disable fetch, so we can use stream clock to access cache
        //-----------------Read from DDLS cache------------------------//
        write_stream(ADDR_RISCV_OFFSET, 8'b00000000);
        // Set data_addr to 32'h00000000
        write_stream(ADDR_DDLS_ADDR, 8'h00);
        write_stream(ADDR_DDLS_ADDR, 8'h00);
        write_stream(ADDR_DDLS_ADDR, 8'h00);
        write_stream(ADDR_DDLS_ADDR, 8'h00);

        // To save simulation time, we just read out the first 4 cache address, for real testing, we will need to read "RISCV_ERRCNT" number of address
        for (int w = 0; w < 4; w = w+1) begin
            write_stream(ADDR_DDLS_OFFSET, 8'b00000000);
            write_stream(ADDR_CACHE_CONTROL, 8'h10);
            
            do begin
                read_stream(ADDR_CACHE_CONTROL);
            end while(RDATA[5] != 1'b1);
            // We have 256bit, so we will need to read 32 times.
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
            read_stream(ADDR_DDLS_DATA_OUT);
        end

        write_stream(ADDR_RISCV_RUN_CYCLE_OFFSET, 8'b00000000);
        // Set data_addr to 32'h00000000

        read_stream(ADDR_RISCV_ERRCNT);
        read_stream(ADDR_RISCV_ERRCNT);
        read_stream(ADDR_RISCV_ERRCNT);
        read_stream(ADDR_RISCV_ERRCNT);
        read_stream(ADDR_RISCV_ERRCNT);
        read_stream(ADDR_RISCV_ERRCNT);
        read_stream(ADDR_RISCV_ERRCNT);
        read_stream(ADDR_RISCV_ERRCNT);
    endtask

    task run_delayed_ddls;
        // Reads in instruction memory file
        $readmemh("/project/chriskim00/jkim/Projects/RAREChip/simulation/post_layout_fullchip/rtl/rare_trigger.mem", instr_data);
        // Instruction cache write
        write_stream(ADDR_RISCV_OFFSET, 8'b00000000);
        write_stream(ADDR_INSTR_ADDR, 8'h00);
        write_stream(ADDR_INSTR_ADDR, 8'h00);
        write_stream(ADDR_INSTR_ADDR, 8'h00);
        write_stream(ADDR_INSTR_ADDR, 8'h00);
        write_stream(ADDR_CACHE_CONTROL, 8'hc0);
        // Start writing instruction data
        for (int w = 0; w < inst_line_number; w = w+1) begin
            write_stream(ADDR_INSTR_DATA_IN, instr_data[w][7:0]); // instr_data[7:0]
            write_stream(ADDR_INSTR_DATA_IN, instr_data[w][15:8]); // instr_data[15:8]
            write_stream(ADDR_INSTR_DATA_IN, instr_data[w][23:16]); // instr_data[23:16]
            write_stream(ADDR_INSTR_DATA_IN, instr_data[w][31:24]); // instr_data[31:24]
        end

        // Enable DCO
        write_stream(ADDR_CLK_CONTROL, 8'b01000000);
        write_stream(ADDR_MAIN_DCO_CONTROL, 8'hd0);
        write_stream(ADDR_MAIN_DCO_EN, 8'h01);

        // DDLS
        write_stream(ADDR_DELAY_SEL, 1);
        write_stream(ADDR_RISCV_RUN_CYCLE_OFFSET, 8'h00);
        

    endtask

    task runcpu;
        // Reads in instruction memory file
        $readmemh("/project/chriskim00/hanzhao/CRYO_RISCV/RARE_CHIP_POST_LAYOUT_11_07/rtl/inst_h.mem_sim", instr_data);
        //---------------Instraction Cache Write----------------------------//
        write_stream(ADDR_RISCV_OFFSET, 8'b00000000); // OFFSET for writing instruction data is 0, means start writing at [7:0]
        // Set instr_addr to 32'h00000000
        write_stream(ADDR_INSTR_ADDR, 8'h00); // instr_addr[7:0] = 8'h00
        write_stream(ADDR_INSTR_ADDR, 8'h00); // instr_addr[15:8] = 8'h00
        write_stream(ADDR_INSTR_ADDR, 8'h00); // instr_addr[23:16] = 8'h00
        write_stream(ADDR_INSTR_ADDR, 8'h00); // instr_addr[31:24] = 8'h00

        write_stream(ADDR_CACHE_CONTROL, 8'hc0); // Stream_sel = 11, writes to both primary and secondary

        // start writing instruction data
        for (int w = 0; w < inst_line_number; w = w+1) begin
            write_stream(ADDR_INSTR_DATA_IN, instr_data[w][7:0]); // instr_data[7:0]
            write_stream(ADDR_INSTR_DATA_IN, instr_data[w][15:8]); // instr_data[15:8]
            write_stream(ADDR_INSTR_DATA_IN, instr_data[w][23:16]); // instr_data[23:16]
            write_stream(ADDR_INSTR_DATA_IN, instr_data[w][31:24]); // instr_data[31:24]
        end
        // end writing instruction.
        //---------------Instraction Cache Write----------------------------//
           
        //---------------Enable DCO----------------------------//
        write_stream(ADDR_CLK_CONTROL,8'b01000000);
        write_stream(ADDR_MAIN_DCO_CONTROL, 8'hd8); // Write value for FINE and COARSE
        write_stream(ADDR_MAIN_DCO_EN, 8'h01);      // Enable DCO
        //---------------Enable DCO----------------------------//
        //---------------Use FPGA Clk----------------------------//
        // write_stream(ADDR_CLK_CONTROL,8'b00000001);
        //---------------Use FPGA Clk----------------------------//

        write_stream(ADDR_DELAY_SEL, 2); // DDLS secondary delay is 2 cycles
        write_stream(ADDR_RISCV_RUN_CYCLE_OFFSET, 8'b00000000); // OFFSET for writing run cycle is 0, means start writing at [7:0], also enalbe run cycle count
        
        write_stream(ADDR_RISCV_RUN_CYCLE, 8'h04); // will run for 4 cycles (64-1)
        write_stream(ADDR_RISCV_RUN_CYCLE, 8'h00);
        write_stream(ADDR_RISCV_RUN_CYCLE, 8'h00);
        write_stream(ADDR_RISCV_RUN_CYCLE, 8'h00);

        write_stream(ADDR_RISCV_OFFSET, 8'b00000000);

        write_stream(ADDR_BOOT_ADDR, 8'h00); // BOOT ADDR, first address instructio is read is at 0x00
        write_stream(ADDR_BOOT_ADDR, 8'h00);
        write_stream(ADDR_BOOT_ADDR, 8'h00);
        write_stream(ADDR_BOOT_ADDR, 8'h00);

        write_stream(ADDR_MTVEC_ADDR, 8'h00); // MTVEC ADDR, not used for this RISCV
        write_stream(ADDR_MTVEC_ADDR, 8'h00);
        write_stream(ADDR_MTVEC_ADDR, 8'h00);
        write_stream(ADDR_MTVEC_ADDR, 8'h00);

        write_stream(ADDR_DEBUG_FREQ_OUT_SEL, 8'b00000000); // Output Mux Select: [0]: FREQ_OUT_SEL ? odometer_vco_pad_out:clk_out_freq_out; [1]DEBUG_OUT_SEL ? secondary:primary

        write_stream(ADDR_RISCV_START, 8'b00000010); // [0] RISCV RESETB [1] FETCH_ENABLE [2] RISCV_CLK_EN [3] DDLS_RESETB
        write_stream(ADDR_RISCV_START, 8'b00001010);
        write_stream(ADDR_RISCV_START, 8'b00001110);
        write_stream(ADDR_RISCV_START, 8'b00001111);
        do begin // wait until done
           read_stream(ADDR_RISCV_STATUS);
        end while(RDATA[0] != 1'b1);     // If we are running two core
        // end while(RDATA[6] != 1'b1);      // If we just run primary
        // end while(RDATA[7] != 1'b1);      // If we just run secondary
        // -----------------------------------------------
        // To check stop and restart.
        // -----------------------------------------------
        // write_stream(ADDR_RISCV_START, 8'b00001011); // 1110 
        // #(clk_period*1);
        // write_stream(ADDR_RISCV_START, 8'b00001111); // 1111
        // do begin // wait until done
        //       read_stream(ADDR_RISCV_STATUS);
        // end while(RDATA[0] != 1'b1);
        // -----------------------------------------------
        // To check stop and restart.
        // -----------------------------------------------
        
        // -----------------------------------------------
        // Read data after cpu execution.
        // -----------------------------------------------
        write_stream(ADDR_RISCV_START, 8'b00001101); //Disable fecth, so the memory can sent data through stream
        write_stream(ADDR_RISCV_OFFSET, 8'b11110100);
        // Set Data_addr to 32'h00000000
        write_stream(ADDR_DATA_ADDR, 8'hFB);// When we read from AXI, make sure shift two bits from the original address in the C code.
        write_stream(ADDR_DATA_ADDR, 8'h01);// i.e. 7EC --> 1FB
        write_stream(ADDR_DATA_ADDR, 8'h00);
        write_stream(ADDR_DATA_ADDR, 8'h00);

        for (int w = 0; w < 4; w = w+1) begin
            write_stream(ADDR_CACHE_CONTROL, 8'h42);
            
            do begin
                read_stream(ADDR_CACHE_CONTROL);
            end while(RDATA[3] != 1'b1);
            
            read_stream(ADDR_DATA_DATA_OUT);
            read_stream(ADDR_DATA_DATA_OUT);
            read_stream(ADDR_DATA_DATA_OUT);
            read_stream(ADDR_DATA_DATA_OUT);
        end
    endtask

    task runcpu_pulse;
        // Reads in instruction memory file
        $readmemh("/project/chriskim00/hanzhao/CRYO_RISCV/RARE_CHIP_POST_LAYOUT_11_07/rtl/inst_h.mem.pulse", instr_data);
        //---------------Instraction Cache Write----------------------------//
        write_stream(ADDR_RISCV_OFFSET, 8'b00000000); // OFFSET for writing instruction data is 0, means start writing at [7:0]
        // Set instr_addr to 32'h00000000
        write_stream(ADDR_INSTR_ADDR, 8'h00); // instr_addr[7:0] = 8'h00
        write_stream(ADDR_INSTR_ADDR, 8'h00); // instr_addr[15:8] = 8'h00
        write_stream(ADDR_INSTR_ADDR, 8'h00); // instr_addr[23:16] = 8'h00
        write_stream(ADDR_INSTR_ADDR, 8'h00); // instr_addr[31:24] = 8'h00

        write_stream(ADDR_CACHE_CONTROL, 8'hc0); // Stream_sel = 11, writes to both primary and secondary

        // start writing instruction data
        for (int w = 0; w < inst_line_number; w = w+1) begin
            write_stream(ADDR_INSTR_DATA_IN, instr_data[w][7:0]); // instr_data[7:0]
            write_stream(ADDR_INSTR_DATA_IN, instr_data[w][15:8]); // instr_data[15:8]
            write_stream(ADDR_INSTR_DATA_IN, instr_data[w][23:16]); // instr_data[23:16]
            write_stream(ADDR_INSTR_DATA_IN, instr_data[w][31:24]); // instr_data[31:24]
        end
        // end writing instruction.
        //---------------Instraction Cache Write----------------------------//
           
        //---------------Enable DCO----------------------------//
        write_stream(ADDR_CLK_CONTROL,8'b01000000);
        write_stream(ADDR_MAIN_DCO_CONTROL, 8'hd8); // Write value for FINE and COARSE
        write_stream(ADDR_MAIN_DCO_EN, 8'h01);      // Enable DCO
        //---------------Enable DCO----------------------------//
        //---------------Use FPGA Clk----------------------------//
        // write_stream(ADDR_CLK_CONTROL,8'b00000001);
        //---------------Use FPGA Clk----------------------------//

        write_stream(ADDR_DELAY_SEL, 0); // DDLS secondary delay is 2 cycles
        write_stream(ADDR_RISCV_RUN_CYCLE_OFFSET, 8'b00000000); // OFFSET for writing run cycle is 0, means start writing at [7:0], also enalbe run cycle count
        
        write_stream(ADDR_RISCV_RUN_CYCLE, 8'h04); // will run for 4 cycles (64-1)
        write_stream(ADDR_RISCV_RUN_CYCLE, 8'h00);
        write_stream(ADDR_RISCV_RUN_CYCLE, 8'h00);
        write_stream(ADDR_RISCV_RUN_CYCLE, 8'h00);

        write_stream(ADDR_RISCV_OFFSET, 8'b00000000);

        write_stream(ADDR_BOOT_ADDR, 8'h00); // BOOT ADDR, first address instructio is read is at 0x00
        write_stream(ADDR_BOOT_ADDR, 8'h00);
        write_stream(ADDR_BOOT_ADDR, 8'h00);
        write_stream(ADDR_BOOT_ADDR, 8'h00);

        write_stream(ADDR_MTVEC_ADDR, 8'h00); // MTVEC ADDR, not used for this RISCV
        write_stream(ADDR_MTVEC_ADDR, 8'h00);
        write_stream(ADDR_MTVEC_ADDR, 8'h00);
        write_stream(ADDR_MTVEC_ADDR, 8'h00);

        write_stream(ADDR_DEBUG_FREQ_OUT_SEL, 8'b00000000); // Output Mux Select: [0]: FREQ_OUT_SEL ? odometer_vco_pad_out:clk_out_freq_out; [1]DEBUG_OUT_SEL ? secondary:primary

        write_stream(ADDR_RISCV_START, 8'b00000010); // [0] RISCV RESETB [1] FETCH_ENABLE [2] RISCV_CLK_EN [3] DDLS_RESETB
        write_stream(ADDR_RISCV_START, 8'b00001010);
        write_stream(ADDR_RISCV_START, 8'b00001110);
        write_stream(ADDR_RISCV_START, 8'b00001111);
        do begin // wait until done
           read_stream(ADDR_RISCV_STATUS);
        end while(RDATA[0] != 1'b1);     // If we are running two core
        // end while(RDATA[6] != 1'b1);      // If we just run primary
        // end while(RDATA[7] != 1'b1);      // If we just run secondary
        // -----------------------------------------------
        // To check stop and restart.
        // -----------------------------------------------
        write_stream(ADDR_RISCV_START, 8'b00001011); // 1110 
        #(clk_period*1);
        write_stream(ADDR_RISCV_START, 8'b00001111); // 1111
        do begin // wait until done
              read_stream(ADDR_RISCV_STATUS);
        end while(RDATA[0] != 1'b1);
        // -----------------------------------------------
        // To check stop and restart.
        // -----------------------------------------------
        
        // -----------------------------------------------
        // Read data after cpu execution.
        // -----------------------------------------------
        write_stream(ADDR_RISCV_START, 8'b00001101); //Disable fecth, so the memory can sent data through stream
        write_stream(ADDR_RISCV_OFFSET, 8'b11110100);
        // Set Data_addr to 32'h00000000
        write_stream(ADDR_DATA_ADDR, 8'hFB);// When we read from AXI, make sure shift two bits from the original address in the C code.
        write_stream(ADDR_DATA_ADDR, 8'h01);// i.e. 7EC --> 1FB
        write_stream(ADDR_DATA_ADDR, 8'h00);
        write_stream(ADDR_DATA_ADDR, 8'h00);

        for (int w = 0; w < 4; w = w+1) begin
            write_stream(ADDR_CACHE_CONTROL, 8'h42);
            
            do begin
                read_stream(ADDR_CACHE_CONTROL);
            end while(RDATA[3] != 1'b1);
            
            read_stream(ADDR_DATA_DATA_OUT);
            read_stream(ADDR_DATA_DATA_OUT);
            read_stream(ADDR_DATA_DATA_OUT);
            read_stream(ADDR_DATA_DATA_OUT);
        end
    endtask

    task odometer_test;
        // Enable DCO
        // There are 21 odometer in each core, so 0-20 are odomerter in primary, 21-41 are odometer in secondary
        write_stream(ADDR_ODOMETER_SEL_7_0, 42);
		write_stream(ADDR_ODOMETER_IN, 8'b00010000);

        // [Stress, AC_DC, SEL_INV, SEL_NAND, SEL_NOR]
        write_stream(ADDR_ODOMETER_SEL_7_0, 5); // RVT
		write_stream(ADDR_ODOMETER_IN, 8'b00011100); //5 
        write_stream(ADDR_ODOMETER_IN, 8'b00010010); //6
        write_stream(ADDR_ODOMETER_IN, 8'b00011001); //7

        write_stream(ADDR_ODOMETER_SEL_7_0, 39); //0-20 for primary 21-41 for secondary Need update
		
		write_stream(ADDR_ODOMETER_IN, 8'b00000010); // S 39
        write_stream(ADDR_ODOMETER_IN, 8'b00001100); // S 40
        write_stream(ADDR_ODOMETER_IN, 8'b00011001); // S 41
		
		

		write_stream(ADDR_ODOMETER_CONTROL, 8'b10000000); // RESETB 1
        write_stream(ADDR_ODOMETER_CONTROL, 8'b10000001); // Load 1
        #(clk_period*20);
		write_stream(ADDR_ODOMETER_CONTROL, 8'b10000010); // Meas Trig 1
        #(clk_period*100);
        write_stream(ADDR_ODOMETER_CONTROL, 8'b10000000); // Meas Trig 0

		#(clk_period*20);

		write_stream(ADDR_ODOMETER_SEL_7_0, 5);
        write_stream(ADDR_ODOMETER_CONTROL, 8'b10000100);
		read_stream(ADDR_ODOMETER_BIT_CNT_7_0);
		read_stream(ADDR_ODOMETER_BIT_CNT_15_8);
        //write_stream(ADDR_ODOMETER_SEL_7_0, 6);
        write_stream(ADDR_ODOMETER_CONTROL, 8'b10000100);
		read_stream(ADDR_ODOMETER_BIT_CNT_7_0);
		read_stream(ADDR_ODOMETER_BIT_CNT_15_8);
        //write_stream(ADDR_ODOMETER_SEL_7_0, 7);
        write_stream(ADDR_ODOMETER_CONTROL, 8'b10000100);
		read_stream(ADDR_ODOMETER_BIT_CNT_7_0);
		read_stream(ADDR_ODOMETER_BIT_CNT_15_8);


        write_stream(ADDR_ODOMETER_SEL_7_0, 39);
        write_stream(ADDR_ODOMETER_CONTROL, 8'b10000100);
		read_stream(ADDR_ODOMETER_BIT_CNT_7_0);
		read_stream(ADDR_ODOMETER_BIT_CNT_15_8);
        //write_stream(ADDR_ODOMETER_SEL_7_0, 40);
        write_stream(ADDR_ODOMETER_CONTROL, 8'b10000100);
		read_stream(ADDR_ODOMETER_BIT_CNT_7_0);
		read_stream(ADDR_ODOMETER_BIT_CNT_15_8);
        //write_stream(ADDR_ODOMETER_SEL_7_0, 41);
        write_stream(ADDR_ODOMETER_CONTROL, 8'b10000100);
		read_stream(ADDR_ODOMETER_BIT_CNT_7_0);
		read_stream(ADDR_ODOMETER_BIT_CNT_15_8);
		

        // [Stress, AC_DC, SEL_INV, SEL_NAND, SEL_NOR]
        write_stream(ADDR_ODOMETER_SEL_7_0, 5); // RVT
		write_stream(ADDR_ODOMETER_IN, 8'b00011100); //5 
        write_stream(ADDR_ODOMETER_IN, 8'b00010010); //6
        write_stream(ADDR_ODOMETER_IN, 8'b00011001); //7

        write_stream(ADDR_ODOMETER_SEL_7_0, 39); //0-20 for primary 21-41 for secondary Need update
		
		write_stream(ADDR_ODOMETER_IN, 8'b00000010); // S 39
        write_stream(ADDR_ODOMETER_IN, 8'b00001100); // S 40
        write_stream(ADDR_ODOMETER_IN, 8'b00011001); // S 41
        //Second measurement
        write_stream(ADDR_ODOMETER_CONTROL, 8'b10000001); // Load 1
        #(clk_period*20);
		write_stream(ADDR_ODOMETER_CONTROL, 8'b10000010); // Meas Trig 1
        #(clk_period*100);
        write_stream(ADDR_ODOMETER_CONTROL, 8'b10000000); // Meas Trig 0

        #(clk_period*20);

		write_stream(ADDR_ODOMETER_SEL_7_0, 5);
        write_stream(ADDR_ODOMETER_CONTROL, 8'b10000100);
		read_stream(ADDR_ODOMETER_BIT_CNT_7_0);
		read_stream(ADDR_ODOMETER_BIT_CNT_15_8);
        write_stream(ADDR_ODOMETER_SEL_7_0, 6);
        write_stream(ADDR_ODOMETER_CONTROL, 8'b10000100);
		read_stream(ADDR_ODOMETER_BIT_CNT_7_0);
		read_stream(ADDR_ODOMETER_BIT_CNT_15_8);
        write_stream(ADDR_ODOMETER_SEL_7_0, 7);
        write_stream(ADDR_ODOMETER_CONTROL, 8'b10000100);
		read_stream(ADDR_ODOMETER_BIT_CNT_7_0);
		read_stream(ADDR_ODOMETER_BIT_CNT_15_8);


        write_stream(ADDR_ODOMETER_SEL_7_0, 39);
        write_stream(ADDR_ODOMETER_CONTROL, 8'b10000100);
		read_stream(ADDR_ODOMETER_BIT_CNT_7_0);
		read_stream(ADDR_ODOMETER_BIT_CNT_15_8);
        write_stream(ADDR_ODOMETER_SEL_7_0, 40);
        write_stream(ADDR_ODOMETER_CONTROL, 8'b10000100);
		read_stream(ADDR_ODOMETER_BIT_CNT_7_0);
		read_stream(ADDR_ODOMETER_BIT_CNT_15_8);
        write_stream(ADDR_ODOMETER_SEL_7_0, 41);
        write_stream(ADDR_ODOMETER_CONTROL, 8'b10000100);
		read_stream(ADDR_ODOMETER_BIT_CNT_7_0);
		read_stream(ADDR_ODOMETER_BIT_CNT_15_8);

        write_stream(ADDR_ODOMETER_CONTROL, 8'b00000000);
    endtask

    task fs_odometer;
        // Enable DCO
        // There are 21 odometer in each core, so 0-20 are odomerter in primary, 21-41 are odometer in secondary
        write_stream(ADDR_ODOMETER_SEL_7_0, 42);
		write_stream(ADDR_ODOMETER_IN, 8'b00010000);
        // Write all the odomter, and at the same time cover all the possible selections.
        for (int w = 0; w < 3; w = w+1) begin
            write_stream(ADDR_ODOMETER_SEL_7_0, 5);
            write_stream(ADDR_ODOMETER_IN, 8'b00011100);
            write_stream(ADDR_ODOMETER_IN, 8'b00010100);
            write_stream(ADDR_ODOMETER_IN, 8'b00001100);
            write_stream(ADDR_ODOMETER_IN, 8'b00000100);
            write_stream(ADDR_ODOMETER_IN, 8'b00011010);
            write_stream(ADDR_ODOMETER_IN, 8'b00010010);
            write_stream(ADDR_ODOMETER_IN, 8'b00001010);
            write_stream(ADDR_ODOMETER_IN, 8'b00000010);
            write_stream(ADDR_ODOMETER_IN, 8'b00011001);
            write_stream(ADDR_ODOMETER_IN, 8'b00010001);
            write_stream(ADDR_ODOMETER_IN, 8'b00001001);
            write_stream(ADDR_ODOMETER_IN, 8'b00000001);
        end

        //Remove resetb, then load the selection, after certain time, enable meas trig.
        write_stream(ADDR_ODOMETER_CONTROL, 8'b10000000); // RESETB 1
        write_stream(ADDR_ODOMETER_CONTROL, 8'b10000001); // Load 1
        #(clk_period*20);;
		write_stream(ADDR_ODOMETER_CONTROL, 8'b10000010); // Meas Trig 1


        write_stream(ADDR_ODOMETER_SEL_7_0, 5);
        write_stream(ADDR_ODOMETER_CONTROL, 8'b10000100);
		read_stream(ADDR_ODOMETER_BIT_CNT_7_0);
		read_stream(ADDR_ODOMETER_BIT_CNT_15_8);
        write_stream(ADDR_ODOMETER_SEL_7_0, 6);
        write_stream(ADDR_ODOMETER_CONTROL, 8'b10000100);
		read_stream(ADDR_ODOMETER_BIT_CNT_7_0);
		read_stream(ADDR_ODOMETER_BIT_CNT_15_8);
        write_stream(ADDR_ODOMETER_SEL_7_0, 7);
        write_stream(ADDR_ODOMETER_CONTROL, 8'b10000100);
		read_stream(ADDR_ODOMETER_BIT_CNT_7_0);
		read_stream(ADDR_ODOMETER_BIT_CNT_15_8);

        //write_stream(ADDR_ODOMETER_SEL_7_0, 7);
		//read_stream(ADDR_ODOMETER_BIT_CNT_7_0);
		//read_stream(ADDR_ODOMETER_BIT_CNT_15_8);

        write_stream(ADDR_ODOMETER_SEL_7_0, 39);
        write_stream(ADDR_ODOMETER_CONTROL, 8'b10000100);
		read_stream(ADDR_ODOMETER_BIT_CNT_7_0);
		read_stream(ADDR_ODOMETER_BIT_CNT_15_8);
        write_stream(ADDR_ODOMETER_SEL_7_0, 40);
        write_stream(ADDR_ODOMETER_CONTROL, 8'b10000100);
		read_stream(ADDR_ODOMETER_BIT_CNT_7_0);
		read_stream(ADDR_ODOMETER_BIT_CNT_15_8);
        write_stream(ADDR_ODOMETER_SEL_7_0, 41);
        write_stream(ADDR_ODOMETER_CONTROL, 8'b10000100);
		read_stream(ADDR_ODOMETER_BIT_CNT_7_0);
		read_stream(ADDR_ODOMETER_BIT_CNT_15_8);
		
        #(clk_period*2);

        write_stream(ADDR_ODOMETER_CONTROL, 8'b00000000);
    endtask

    task heater_test;
        write_stream(ADDR_HEATER_P_SEL, 8'b00000000);
        write_stream(ADDR_HEATER_P_IN, 8'hFF);//Write 8 heater at a time
        write_stream(ADDR_HEATER_P_IN, 8'hFF);
        write_stream(ADDR_HEATER_P_IN, 8'h11);
        write_stream(ADDR_HEATER_P_IN, 8'h33);

        write_stream(ADDR_HEATER_S_SEL, 8'b00000000);
        write_stream(ADDR_HEATER_S_IN, 8'h22);
        write_stream(ADDR_HEATER_S_IN, 8'h88);
        
        write_stream(ADDR_HEATER_CONTROL, 8'h80); // Global enable on

        #(clk_period*2);
        
        write_stream(ADDR_HEATER_CONTROL, 8'h00); // Global enable off
        write_stream(ADDR_HEATER_CONTROL, 8'h07); // Heater Load all
        write_stream(ADDR_HEATER_CONTROL, 8'h80); // Global enable on
        
        #(clk_period*200);

        write_stream(ADDR_HEATER_P_SEL, 8'b00000000);
        write_stream(ADDR_HEATER_P_IN, 8'h17);
        write_stream(ADDR_HEATER_P_IN, 8'h17);
        write_stream(ADDR_HEATER_P_IN, 8'h17);
        write_stream(ADDR_HEATER_P_IN, 8'h17);
        //write_stream(ADDR_HEATER_CONTROL, 8'h00); // Global enable off
        write_stream(ADDR_HEATER_CONTROL, 8'h87); // Heater Load all
        //write_stream(ADDR_HEATER_CONTROL, 8'h80); // Global enable on
        // write_stream(ADDR_HEATER_CONTROL, 8'h07); // Heater Load all

        #(clk_period*200);
        write_stream(ADDR_HEATER_CONTROL, 8'hc0);// Enable both heater and sensor --> enable sensor earlier than actual input.
        #(clk_period*10);
        write_stream(ADDR_TEMP_SENSOR_SEL, 8'h03);// Sensor is one-hot decoded
        #(clk_period*10);
        write_stream(ADDR_TEMP_SENSOR_SEL, 8'h11);//Select last from Primary
        #(clk_period*10);
        write_stream(ADDR_TEMP_SENSOR_SEL, 8'h23);//Select last from Secondary
        #(clk_period*10);
        write_stream(ADDR_TEMP_SENSOR_SEL, 8'h25);//Select out of range
        #(clk_period*10);
        write_stream(ADDR_HEATER_CONTROL, 8'h80);
    endtask

    task dco_test;

        #clk_period;
        write_stream(ADDR_CLK_CONTROL,8'b01000000);
        write_stream(ADDR_MAIN_DCO_CONTROL, 8'hFF); // Write value for FINE and COARSE
        write_stream(ADDR_MAIN_DCO_EN, 8'h01);      // Enable DCO
        #(clk_period*1);
        write_stream(ADDR_MAIN_DCO_EN, 8'h00);
        
        #(clk_period*5);

        write_stream(ADDR_MAIN_DCO_CONTROL, 8'h00);
        write_stream(ADDR_MAIN_DCO_EN, 8'h01);

        #(clk_period*5);


        write_stream(ADDR_DEBUG_FREQ_OUT_SEL, 8'b00000000);
        write_stream(ADDR_CLK_CONTROL, 8'h02);

        #(clk_period*100);
    endtask

    task dco_count_test;

        write_stream(ADDR_MAIN_DCO_CONTROL, 8'hFF);// Write value for FINE and COARSE
        write_stream(ADDR_MAIN_DCO_EN, 8'h01);
        
        // Clock frequency measurement
        write_stream(ADDR_CLK_CONTROL, 8'h08);
        counter_en = 1'b1;
        write_stream(ADDR_CLK_CONTROL, 8'h18); // Start
        do begin // wait until done
            read_stream(ADDR_CLK_CONTROL);
        end while(RDATA[5] != 1'b1);
        counter_en = 1'b0;
        #(clk_period*5);

    endtask

    task odo_dco_test;
        #clk_period;
        write_stream(ADDR_ODOMETER_SEL_7_0, 42);
		write_stream(ADDR_ODOMETER_IN, 8'b00010011);
        write_stream(ADDR_DEBUG_FREQ_OUT_SEL, 8'b00000001);
        write_stream(ADDR_ODOMETER_CONTROL, 8'b10000000); // RESETB 1
        write_stream(ADDR_ODOMETER_CONTROL, 8'b10000001); // Load 1
        #(clk_period*10000);
    endtask

    task fs_cpu;
        
        write_stream(ADDR_MAIN_DCO_CONTROL, 8'h00);      //  Program DCO Freq


        write_stream(ADDR_HEATER_P_SEL, 8'b00000000);   //  Program heater location and start heater
        write_stream(ADDR_HEATER_P_IN, 8'hFF);          //  Write 8 heater at a time
        write_stream(ADDR_HEATER_P_IN, 8'hFF);
        write_stream(ADDR_HEATER_P_IN, 8'h11);
        write_stream(ADDR_HEATER_P_IN, 8'h33);
        write_stream(ADDR_HEATER_CONTROL, 8'h07);       //  Heater Load all
        write_stream(ADDR_HEATER_CONTROL, 8'h80);       //  Global enable on

        write_stream(ADDR_TEMP_SENSOR_SEL, 8'h03);      //  Select Temp_sensor to check
        write_stream(ADDR_HEATER_CONTROL, 8'hc0);       // Enable both heater and sensor


        //After we reach a certain temperature, start loading the instruction

        // Reads in instruction memory file
        $readmemh("/project/chriskim00/hanzhao/CRYO_RISCV/RARE_CHIP_POST_LAYOUT_11_07/rtl/inst_h.mem_sim", instr_data);
        //---------------Instraction Cache Write----------------------------//
        write_stream(ADDR_RISCV_OFFSET, 8'b00000000); // OFFSET for writing instruction data is 0, means start writing at [7:0]
        // Set instr_addr to 32'h00000000
        write_stream(ADDR_INSTR_ADDR, 8'h00); // instr_addr[7:0] = 8'h00
        write_stream(ADDR_INSTR_ADDR, 8'h00); // instr_addr[15:8] = 8'h00
        write_stream(ADDR_INSTR_ADDR, 8'h00); // instr_addr[23:16] = 8'h00
        write_stream(ADDR_INSTR_ADDR, 8'h00); // instr_addr[31:24] = 8'h00

        write_stream(ADDR_CACHE_CONTROL, 8'hC0); // Stream_sel = 11, writes to both primary and secondary

        // start writing instruction data
        for (int w = 0; w < inst_line_number; w = w+1) begin
            write_stream(ADDR_INSTR_DATA_IN, instr_data[w][7:0]); // instr_data[7:0]
            write_stream(ADDR_INSTR_DATA_IN, instr_data[w][15:8]); // instr_data[15:8]
            write_stream(ADDR_INSTR_DATA_IN, instr_data[w][23:16]); // instr_data[23:16]
            write_stream(ADDR_INSTR_DATA_IN, instr_data[w][31:24]); // instr_data[31:24]
        end
        // end writing instruction.
        //---------------Instraction Cache Write----------------------------//

        //---------------Enable DCO----------------------------//
        write_stream(ADDR_CLK_CONTROL,8'b01000000);
        write_stream(ADDR_MAIN_DCO_EN, 8'h01);      // Enable DCO
        //---------------Enable DCO----------------------------//
        //---------------Use FPGA Clk----------------------------//
        // write_stream(ADDR_CLK_CONTROL,8'b00000001);
        //---------------Use FPGA Clk----------------------------//

        write_stream(ADDR_DELAY_SEL, 0); // DDLS secondary delay is 2 cycles
        write_stream(ADDR_RISCV_RUN_CYCLE_OFFSET, 8'b00000000); // OFFSET for writing run cycle is 0, means start writing at [7:0], also enalbe run cycle count
        
        write_stream(ADDR_RISCV_RUN_CYCLE, 8'h04); // will run for 4 cycles (64-1)
        write_stream(ADDR_RISCV_RUN_CYCLE, 8'h00);
        write_stream(ADDR_RISCV_RUN_CYCLE, 8'h00);
        write_stream(ADDR_RISCV_RUN_CYCLE, 8'h00);

        write_stream(ADDR_RISCV_OFFSET, 8'b00000000);

        write_stream(ADDR_BOOT_ADDR, 8'h00); // BOOT ADDR, first address instructio is read is at 0x00
        write_stream(ADDR_BOOT_ADDR, 8'h00);
        write_stream(ADDR_BOOT_ADDR, 8'h00);
        write_stream(ADDR_BOOT_ADDR, 8'h00);

        write_stream(ADDR_MTVEC_ADDR, 8'h00); // MTVEC ADDR, not used for this RISCV
        write_stream(ADDR_MTVEC_ADDR, 8'h00);
        write_stream(ADDR_MTVEC_ADDR, 8'h00);
        write_stream(ADDR_MTVEC_ADDR, 8'h00);

        write_stream(ADDR_DEBUG_FREQ_OUT_SEL, 8'b00000000); // Output Mux Select: [0]: FREQ_OUT_SEL ? odometer_vco_pad_out:clk_out_freq_out; [1]DEBUG_OUT_SEL ? secondary:primary

        write_stream(ADDR_RISCV_START, 8'b00000010); // 0010 Fetch enable, now riscv is able to fetch data from instruction cache
        write_stream(ADDR_RISCV_START, 8'b00001010); // 1010 DDLS_RESETB deassert
        write_stream(ADDR_RISCV_START, 8'b00001110); // RESET

        write_stream(ADDR_RISCV_START, 8'b00001111); // 1111

        //This is an extrem case: change DCO freq while execution.
        // #(clk_period*5);
        // write_stream(ADDR_MAIN_DCO_CONTROL, 8'hB0);

        do begin // wait until done
           read_stream(ADDR_RISCV_STATUS);
        end while(RDATA[0] != 1'b1);     // If we are running two core
        //end while(RDATA[6] != 1'b1);      // If we just run primary
        
        // -----------------------------------------------
        // To check stop and restart.
        // -----------------------------------------------
        // write_stream(ADDR_RISCV_START, 8'b00001011); // 1110 
        // write_stream(ADDR_RISCV_RUN_CYCLE_OFFSET, 8'b00000000);//Remove the run count limit this time.
        // #(clk_period*1);
        // write_stream(ADDR_RISCV_START, 8'b00001111); // 1111
        // do begin // wait until done
        //    read_stream(ADDR_RISCV_STATUS);
        // end while(RDATA[0] != 1'b1);
        // -----------------------------------------------
        // To check stop and restart.
        // -----------------------------------------------
        
        // -----------------------------------------------
        // Read data after cpu execution.
        // -----------------------------------------------
        write_stream(ADDR_RISCV_START, 8'b00001101); //Disable fecth, so the memory can sent data through stream
        write_stream(ADDR_RISCV_OFFSET, 8'b11110100);
        // Set Data_addr to 32'h00000000
        write_stream(ADDR_DATA_ADDR, 8'hFB);// When we read from AXI, make sure shift two bits from the original address in the C code.
        write_stream(ADDR_DATA_ADDR, 8'h01);// i.e. 7EC --> 1FB
        write_stream(ADDR_DATA_ADDR, 8'h00);
        write_stream(ADDR_DATA_ADDR, 8'h00);

        for (int w = 0; w < 4; w = w+1) begin
            write_stream(ADDR_CACHE_CONTROL, 8'h42);
            
            do begin
                read_stream(ADDR_CACHE_CONTROL);
            end while(RDATA[3] != 1'b1);
            
            read_stream(ADDR_DATA_DATA_OUT);
            read_stream(ADDR_DATA_DATA_OUT);
            read_stream(ADDR_DATA_DATA_OUT);
            read_stream(ADDR_DATA_DATA_OUT);
        end
    endtask

endmodule
