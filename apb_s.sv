module apb_s #(
    parameter ADDR_WIDTH  = 32,
    parameter DATA_WIDTH  = 8,
    parameter WAIT_CYCLES = 1
)
(
    input                        pclk,
    input                        presetn,
    input  [ADDR_WIDTH-1:0]      paddr,
    input                        psel,
    input                        penable,
    input  [DATA_WIDTH-1:0]      pwdata,
    input                        pwrite,

    output reg [DATA_WIDTH-1:0]  prdata,
    output reg                   pready,
    output                       pslverr
);

    // APB transfer states
    localparam [1:0] idle = 0, write = 1, read = 2;

    reg [DATA_WIDTH-1:0] mem[16];
    reg [1:0] state, nstate;
    reg [3:0] wait_cnt;

    bit addr_err;

    // Update the current state
    always @(posedge pclk, negedge presetn) begin
        if (presetn == 1'b0)
            state <= idle;
        else
            state <= nstate;
    end

    // Count the wait cycles during an access
    always @(posedge pclk, negedge presetn) begin
        if (presetn == 1'b0)
            wait_cnt <= 0;
        else if (state == idle)
            wait_cnt <= 0;
        else if ((state == write || state == read) && psel && penable && (wait_cnt < WAIT_CYCLES))
            wait_cnt <= wait_cnt + 1'b1;
    end

    // Select the next state and drive the outputs
    always @(*) begin
        nstate = state;
        pready = 1'b0;
        prdata = {DATA_WIDTH{1'b0}};

        case (state)
            idle: begin
                if (psel == 1'b1 && pwrite == 1'b1)
                    nstate = write;
                else if (psel == 1'b1 && pwrite == 1'b0)
                    nstate = read;
                else
                    nstate = idle;
            end

            write: begin
                if (psel == 1'b1 && penable == 1'b1) begin
                    if (wait_cnt >= WAIT_CYCLES) begin
                        pready = 1'b1;
                        nstate = idle;
                    end
                    else begin
                        pready = 1'b0;
                        nstate = write;
                    end
                end
            end

            read: begin
                if (psel == 1'b1 && penable == 1'b1) begin
                    if (wait_cnt >= WAIT_CYCLES) begin
                        pready = 1'b1;
                        if (!addr_err)
                            prdata = mem[paddr];
                        else
                            prdata = {DATA_WIDTH{1'b0}};
                        nstate = idle;
                    end
                    else begin
                        pready = 1'b0;
                        nstate = read;
                    end
                end
            end

            default: begin
                nstate = idle;
            end
        endcase
    end

    // Store data when a write transfer completes
    always @(posedge pclk) begin
        if (state == write && psel == 1'b1 && penable == 1'b1 && (wait_cnt >= WAIT_CYCLES) && !addr_err)
            mem[paddr] <= pwdata;
    end

    // Flag addresses outside the memory range
    assign addr_err = ((state == write) || (state == read)) && (paddr > 15);
    assign pslverr = (psel == 1'b1 && penable == 1'b1) ? addr_err : 1'b0;

endmodule
