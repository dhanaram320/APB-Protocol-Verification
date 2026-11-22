interface apb_if();

    logic        pclk;
    logic        presetn;
    logic [31:0] paddr;
    logic        psel;
    logic        penable;
    logic [7:0]  pwdata;
    logic        pwrite;

    logic [7:0]  prdata;
    logic        pready;
    logic        pslverr;

    // Check that ACCESS stays active until the slave is ready
    property penable_stable_till_ready;
        @(posedge pclk) disable iff (!presetn)
            (penable && !pready) |=> penable;
    endproperty
    assert property (penable_stable_till_ready)
        else $error("[ASSERT] penable dropped before pready");

    // Keep the address and data stable while waiting
    property addr_data_stable_till_ready;
        @(posedge pclk) disable iff (!presetn)
            (penable && !pready) |=> ($stable(paddr) && $stable(pwdata));
    endproperty
    assert property (addr_data_stable_till_ready)
        else $error("[ASSERT] paddr/pwdata changed during ACCESS phase");

    // PENABLE must not be high unless PSEL is high
    property penable_needs_psel;
        @(posedge pclk) disable iff (!presetn)
            penable |-> psel;
    endproperty
    assert property (penable_needs_psel)
        else $error("[ASSERT] penable asserted without psel");

endinterface

// Holds the values for one APB transaction
class transaction;

    rand logic [31:0] paddr;
    rand logic [7:0] pwdata;
    rand bit psel;
    rand bit penable;
    randc bit pwrite;
    bit [7:0] prdata;
    bit pready;
    bit pslverr;

    constraint addr_c {
        paddr >= 0;
        paddr <= 20;
    }

    constraint data_c {
        pwdata >= 0;
        pwdata <= 270;
    }

    function void display(input string tag);
        $display("[%0s] :  paddr:%0d  pwdata:%0d pwrite:%0b  prdata:%0d pslverr:%0b @ %0t",
                 tag, paddr, pwdata, pwrite, prdata, pslverr, $time);
    endfunction

endclass

// Creates transactions for the driver
class generator;

    transaction tr;
    mailbox #(transaction) mbx;
    int count = 0;

    event nextdrv;
    event nextsco;
    event done;

    function new(mailbox #(transaction) mbx);
        this.mbx = mbx;
        tr = new();
    endfunction

    task run();
        repeat (count) begin
            assert (tr.randomize()) else $error("Randomization failed");
            mbx.put(tr);
            tr.display("GEN");
            @(nextdrv);
            @(nextsco);
        end
        -> done;
    endtask

endclass

// Drives read and write transfers on the APB interface
class driver;

    virtual apb_if vif;
    mailbox #(transaction) mbx;
    transaction datac;

    event nextdrv;

    function new(mailbox #(transaction) mbx);
        this.mbx = mbx;
    endfunction

    task reset();
        vif.presetn <= 1'b0;
        vif.psel    <= 1'b0;
        vif.penable <= 1'b0;
        vif.pwdata  <= 0;
        vif.paddr   <= 0;
        vif.pwrite  <= 1'b0;
        repeat (5) @(posedge vif.pclk);
        vif.presetn <= 1'b1;
        $display("[DRV] : RESET DONE");
        $display("----------------------------------------------------------------------------");
    endtask

    task run();
        forever begin
            mbx.get(datac);
            @(posedge vif.pclk);

            if (datac.pwrite == 1) begin
                vif.psel    <= 1'b1;
                vif.penable <= 1'b0;
                vif.pwdata  <= datac.pwdata;
                vif.paddr   <= datac.paddr;
                vif.pwrite  <= 1'b1;
                @(posedge vif.pclk);
                vif.penable <= 1'b1;
                @(posedge vif.pclk);

                while (vif.pready == 1'b0)
                    @(posedge vif.pclk);

                vif.psel    <= 1'b0;
                vif.penable <= 1'b0;
                vif.pwrite  <= 1'b0;
                datac.display("DRV");
                -> nextdrv;
            end
            else if (datac.pwrite == 0) begin
                vif.psel    <= 1'b1;
                vif.penable <= 1'b0;
                vif.pwdata  <= 0;
                vif.paddr   <= datac.paddr;
                vif.pwrite  <= 1'b0;
                @(posedge vif.pclk);
                vif.penable <= 1'b1;
                @(posedge vif.pclk);

                while (vif.pready == 1'b0)
                    @(posedge vif.pclk);

                vif.psel    <= 1'b0;
                vif.penable <= 1'b0;
                vif.pwrite  <= 1'b0;
                datac.display("DRV");
                -> nextdrv;
            end
        end
    endtask

endclass

// Collects the result of each completed transfer
class monitor;

    virtual apb_if vif;
    mailbox #(transaction) mbx;
    transaction tr;

    function new(mailbox #(transaction) mbx);
        this.mbx = mbx;
    endfunction

    task run();
        tr = new();
        forever begin
            @(posedge vif.pclk);
            if (vif.pready) begin
                tr.pwdata  = vif.pwdata;
                tr.paddr   = vif.paddr;
                tr.pwrite  = vif.pwrite;
                tr.prdata  = vif.prdata;
                tr.pslverr = vif.pslverr;
                @(posedge vif.pclk);
                tr.display("MON");
                mbx.put(tr);
            end
        end
    endtask

endclass

// Checks read data and records functional coverage
class scoreboard;

    mailbox #(transaction) mbx;
    transaction tr;
    event nextsco;

    bit [7:0] pwdata[16] = '{default:0};
    bit [7:0] rdata;
    int err = 0;

    covergroup cg;
        option.per_instance = 1;

        cp_addr : coverpoint tr.paddr {
            bins low_addr  = {[0:7]};
            bins high_addr = {[8:15]};
        }

        cp_wr : coverpoint tr.pwrite {
            bins write = {1'b1};
            bins read  = {1'b0};
        }

        cp_err : coverpoint tr.pslverr {
            bins no_err = {1'b0};
            bins err    = {1'b1};
        }
    endgroup

    function new(mailbox #(transaction) mbx);
        this.mbx = mbx;
        cg = new();
    endfunction

    task run();
        forever begin
            mbx.get(tr);
            tr.display("SCO");
            cg.sample();

            if ((tr.pwrite == 1'b1) && (tr.pslverr == 1'b0)) begin
                pwdata[tr.paddr] = tr.pwdata;
                $display("[SCO] : DATA STORED DATA : %0d ADDR: %0d", tr.pwdata, tr.paddr);
            end
            else if ((tr.pwrite == 1'b0) && (tr.pslverr == 1'b0)) begin
                rdata = pwdata[tr.paddr];
                if (tr.prdata == rdata)
                    $display("[SCO] : Data Matched");
                else begin
                    err++;
                    $display("[SCO] : Data Mismatched");
                end
            end
            else if (tr.pslverr == 1'b1) begin
                $display("[SCO] : SLV ERROR DETECTED");
            end

            $display("---------------------------------------------------------------------------------------------------");
            -> nextsco;
        end
    endtask

endclass

// Connects the testbench components
class environment;

    generator gen;
    driver drv;
    monitor mon;
    scoreboard sco;

    event nextgd;
    event nextgs;

    mailbox #(transaction) gdmbx;
    mailbox #(transaction) msmbx;

    virtual apb_if vif;

    function new(virtual apb_if vif);
        gdmbx = new();
        gen = new(gdmbx);
        drv = new(gdmbx);

        msmbx = new();
        mon = new(msmbx);
        sco = new(msmbx);

        this.vif = vif;
        drv.vif = this.vif;
        mon.vif = this.vif;

        gen.nextsco = nextgs;
        sco.nextsco = nextgs;

        gen.nextdrv = nextgd;
        drv.nextdrv = nextgd;
    endfunction

    task pre_test();
        drv.reset();
    endtask

    task test();
        fork
            gen.run();
            drv.run();
            mon.run();
            sco.run();
        join_any
    endtask

    task post_test();
        wait (gen.done.triggered);
        $display("----Total number of Mismatch : %0d------", sco.err);
        $display("----Functional coverage achieved : %0.2f%%------", sco.cg.get_coverage());
        $finish();
    endtask

    task run();
        pre_test();
        test();
        post_test();
    endtask

endclass

module tb;

    apb_if vif();

    apb_s dut (
        vif.pclk,
        vif.presetn,
        vif.paddr,
        vif.psel,
        vif.penable,
        vif.pwdata,
        vif.pwrite,
        vif.prdata,
        vif.pready,
        vif.pslverr
    );

    initial begin
        vif.pclk <= 0;
    end

    always #10 vif.pclk <= ~vif.pclk;

    environment env;

    initial begin
        env = new(vif);
        env.gen.count = 20;
        env.run();
    end

    initial begin
        $dumpfile("dump.vcd");
        $dumpvars;
    end

endmodule
