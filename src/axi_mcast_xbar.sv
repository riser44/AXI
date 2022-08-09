// Copyright (c) 2022 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Authors:
// - Luca Colagrande <colluca@iis.ee.ethz.ch>
// Based on:
// - axi_xbar.sv

// axi_multicast_xbar: Multicast-enabled fully-connected AXI4+ATOP crossbar with an arbitrary number
// of slave and master ports.
// See `doc/axi_xbar.md` for the documentation, including the definition of parameters and ports.
module axi_mcast_xbar
import cf_math_pkg::idx_width;
#(
  parameter axi_pkg::xbar_cfg_t Cfg                                   = '0,
  parameter bit  ATOPs                                                = 1'b1,
  parameter bit [Cfg.NoSlvPorts-1:0][Cfg.NoMstPorts-1:0] Connectivity = '1,
  parameter type slv_aw_chan_t                                        = logic,
  parameter type mst_aw_chan_t                                        = logic,
  parameter type w_chan_t                                             = logic,
  parameter type slv_b_chan_t                                         = logic,
  parameter type mst_b_chan_t                                         = logic,
  parameter type slv_ar_chan_t                                        = logic,
  parameter type mst_ar_chan_t                                        = logic,
  parameter type slv_r_chan_t                                         = logic,
  parameter type mst_r_chan_t                                         = logic,
  parameter type slv_req_t                                            = logic,
  parameter type slv_resp_t                                           = logic,
  parameter type mst_req_t                                            = logic,
  parameter type mst_resp_t                                           = logic,
  parameter type rule_t                                               = axi_pkg::xbar_rule_64_t
) (
  input  logic                            clk_i,
  input  logic                            rst_ni,
  input  logic                            test_i,
  input  slv_req_t  [Cfg.NoSlvPorts-1:0]  slv_ports_req_i,
  output slv_resp_t [Cfg.NoSlvPorts-1:0]  slv_ports_resp_o,
  output mst_req_t  [Cfg.NoMstPorts-1:0]  mst_ports_req_o,
  input  mst_resp_t [Cfg.NoMstPorts-1:0]  mst_ports_resp_i,
  input  rule_t     [Cfg.NoAddrRules-1:0] addr_map_i
);

  typedef logic [Cfg.AxiAddrWidth-1:0] addr_t;
  // to account for the decoding error slave
  typedef logic [(Cfg.NoMstPorts+1)-1:0] mst_port_mask_t;

  // signals from the axi_demuxes, one index more for decode error
  slv_req_t  [Cfg.NoSlvPorts-1:0][Cfg.NoMstPorts:0]  slv_reqs;
  slv_resp_t [Cfg.NoSlvPorts-1:0][Cfg.NoMstPorts:0]  slv_resps;

  // workaround for issue #133 (problem with vsim 10.6c)
  localparam int unsigned cfg_NoMstPorts = Cfg.NoMstPorts;

  // signals into the axi_muxes, are of type slave as the multiplexer extends the ID
  slv_req_t  [Cfg.NoMstPorts-1:0][Cfg.NoSlvPorts-1:0] mst_reqs;
  slv_resp_t [Cfg.NoMstPorts-1:0][Cfg.NoSlvPorts-1:0] mst_resps;

  for (genvar i = 0; i < Cfg.NoSlvPorts; i++) begin : gen_slv_port_demux
    logic  [Cfg.NoMstPorts-1:0] dec_aw_select, dec_ar_select;
    addr_t [Cfg.NoMstPorts-1:0] dec_aw_addr,   dec_aw_mask;
    logic                       dec_aw_valid,  dec_aw_error;
    logic                       dec_ar_valid,  dec_ar_error;
    mst_port_mask_t             slv_aw_select, slv_ar_select;
    addr_t [Cfg.NoMstPorts:0]   slv_aw_addr,   slv_aw_mask;

    multiaddr_decode #(
      .NoRules(Cfg.NoMstPorts),
      .addr_t (addr_t),
      .rule_t (rule_t)
    ) i_axi_aw_decode (
      .addr_i     (slv_ports_req_i[i].aw.addr),
      .mask_i     (slv_ports_req_i[i].aw.user.mcast),
      .addr_map_i (addr_map_i),
      .select_o   (dec_aw_select),
      .addr_o     (dec_aw_addr),
      .mask_o     (dec_aw_mask),
      .dec_valid_o(dec_aw_valid),
      .dec_error_o(dec_aw_error)
    );

    multiaddr_decode #(
      .NoRules(Cfg.NoMstPorts),
      .addr_t (addr_t),
      .rule_t (rule_t)
    ) i_axi_ar_decode (
      .addr_i     (slv_ports_req_i[i].ar.addr),
      .mask_i     ('0),
      .addr_map_i (addr_map_i),
      .select_o   (dec_ar_select),
      .addr_o     (/* Unused */),
      .mask_o     (/* Unused */),
      .dec_valid_o(dec_ar_valid),
      .dec_error_o(dec_ar_error)
    );

    // decerror slave
    assign slv_aw_select = (dec_aw_error) ?
        {1'b1, {Cfg.NoMstPorts{1'b0}}} : {1'b0, dec_aw_select};
    assign slv_ar_select = (dec_ar_error) ?
        {1'b1, {Cfg.NoMstPorts{1'b0}}} : {1'b0, dec_ar_select};
    assign slv_aw_addr = {'0, dec_aw_addr};
    assign slv_aw_mask = {'0, dec_aw_mask};

    axi_mcast_demux #(
      .AxiIdWidth (Cfg.AxiIdWidthSlvPorts), // ID Width
      .AtopSupport(ATOPs),
      .aw_addr_t  (addr_t),                 // AW Address Type
      .aw_chan_t  (slv_aw_chan_t),          // AW Channel Type
      .w_chan_t   (w_chan_t),               //  W Channel Type
      .b_chan_t   (slv_b_chan_t),           //  B Channel Type
      .ar_chan_t  (slv_ar_chan_t),          // AR Channel Type
      .r_chan_t   (slv_r_chan_t),           //  R Channel Type
      .axi_req_t  (slv_req_t),
      .axi_resp_t (slv_resp_t),
      .NoMstPorts (Cfg.NoMstPorts + 1),
      .MaxTrans   (Cfg.MaxMstTrans),
      .AxiLookBits(Cfg.AxiIdUsedSlvPorts),
      .UniqueIds  (Cfg.UniqueIds),
      .FallThrough(Cfg.FallThrough),
      .SpillAw    (Cfg.LatencyMode[9]),
      .SpillW     (Cfg.LatencyMode[8]),
      .SpillB     (Cfg.LatencyMode[7]),
      .SpillAr    (Cfg.LatencyMode[6]),
      .SpillR     (Cfg.LatencyMode[5])
    ) i_axi_demux (
      .clk_i,   // Clock
      .rst_ni,  // Asynchronous reset active low
      .test_i,  // Testmode enable
      .slv_req_i      (slv_ports_req_i[i]),
      .slv_aw_select_i(slv_aw_select),
      .slv_ar_select_i(slv_ar_select),
      .slv_aw_addr_i  (slv_aw_addr),
      .slv_aw_mask_i  (slv_aw_mask),
      .slv_resp_o     (slv_ports_resp_o[i]),
      .mst_reqs_o     (slv_reqs[i]),
      .mst_resps_i    (slv_resps[i])
    );

    axi_err_slv #(
      .AxiIdWidth(Cfg.AxiIdWidthSlvPorts ),
      .axi_req_t (slv_req_t),
      .axi_resp_t(slv_resp_t),
      .Resp      (axi_pkg::RESP_DECERR),
      .ATOPs     (ATOPs),
      .MaxTrans  (4) // Transactions terminate at this slave, so minimize
                     // resource consumption by accepting only a few
                     // transactions at a time.
    ) i_axi_err_slv (
      .clk_i,   // Clock
      .rst_ni,  // Asynchronous reset active low
      .test_i,  // Testmode enable
      // slave port
      .slv_req_i (slv_reqs[i][Cfg.NoMstPorts]),
      .slv_resp_o(slv_resps[i][cfg_NoMstPorts])
    );
  end

  // cross all channels
  for (genvar i = 0; i < Cfg.NoSlvPorts; i++) begin : gen_xbar_slv_cross
    for (genvar j = 0; j < Cfg.NoMstPorts; j++) begin : gen_xbar_mst_cross
      if (Connectivity[i][j]) begin : gen_connection
        assign mst_reqs[j][i]  = slv_reqs[i][j];
        assign slv_resps[i][j] = mst_resps[j][i];

      end else begin : gen_no_connection
        assign mst_reqs[j][i] = '0;
        axi_err_slv #(
          .AxiIdWidth(Cfg.AxiIdWidthSlvPorts),
          .axi_req_t (slv_req_t),
          .axi_resp_t(slv_resp_t),
          .Resp      (axi_pkg::RESP_DECERR),
          .ATOPs     (ATOPs),
          .MaxTrans  (1)
        ) i_axi_err_slv (
          .clk_i,
          .rst_ni,
          .test_i,
          .slv_req_i (slv_reqs[i][j]),
          .slv_resp_o(slv_resps[i][j])
        );
      end
    end
  end

  for (genvar i = 0; i < Cfg.NoMstPorts; i++) begin : gen_mst_port_mux
    axi_mux #(
      .SlvAxiIDWidth(Cfg.AxiIdWidthSlvPorts ), // ID width of the slave ports
      .slv_aw_chan_t(slv_aw_chan_t),           // AW Channel Type, slave ports
      .mst_aw_chan_t(mst_aw_chan_t),           // AW Channel Type, master port
      .w_chan_t     (w_chan_t),                //  W Channel Type, all ports
      .slv_b_chan_t (slv_b_chan_t),            //  B Channel Type, slave ports
      .mst_b_chan_t (mst_b_chan_t),            //  B Channel Type, master port
      .slv_ar_chan_t(slv_ar_chan_t),           // AR Channel Type, slave ports
      .mst_ar_chan_t(mst_ar_chan_t),           // AR Channel Type, master port
      .slv_r_chan_t (slv_r_chan_t),            //  R Channel Type, slave ports
      .mst_r_chan_t (mst_r_chan_t),            //  R Channel Type, master port
      .slv_req_t    (slv_req_t),
      .slv_resp_t   (slv_resp_t),
      .mst_req_t    (mst_req_t),
      .mst_resp_t   (mst_resp_t),
      .NoSlvPorts   (Cfg.NoSlvPorts),          // Number of Masters for the module
      .MaxWTrans    (Cfg.MaxSlvTrans),
      .FallThrough  (Cfg.FallThrough),
      .SpillAw      (Cfg.LatencyMode[4]),
      .SpillW       (Cfg.LatencyMode[3]),
      .SpillB       (Cfg.LatencyMode[2]),
      .SpillAr      (Cfg.LatencyMode[1]),
      .SpillR       (Cfg.LatencyMode[0])
    ) i_axi_mux (
      .clk_i,   // Clock
      .rst_ni,  // Asynchronous reset active low
      .test_i,  // Test Mode enable
      .slv_reqs_i (mst_reqs[i]),
      .slv_resps_o(mst_resps[i]),
      .mst_req_o  (mst_ports_req_o[i]),
      .mst_resp_i (mst_ports_resp_i[i])
    );
  end

  // pragma translate_off
  `ifndef VERILATOR
  `ifndef XSIM
  initial begin : check_params
    id_slv_req_ports: assert ($bits(slv_ports_req_i[0].aw.id ) == Cfg.AxiIdWidthSlvPorts) else
      $fatal(1, $sformatf("Slv_req and aw_chan id width not equal."));
    id_slv_resp_ports: assert ($bits(slv_ports_resp_o[0].r.id) == Cfg.AxiIdWidthSlvPorts) else
      $fatal(1, $sformatf("Slv_req and aw_chan id width not equal."));
    addr_slv_req_ports: assert ($bits(slv_ports_req_i[0].aw.addr) == Cfg.AxiAddrWidth) else
      $fatal(1, $sformatf("Slv_req and aw_addr width not equal."));
    addr_mst_req_ports: assert ($bits(mst_ports_req_o[0].aw.addr) == Cfg.AxiAddrWidth) else
      $fatal(1, $sformatf("Mst_req and aw_addr width not equal."));
  end
  `endif
  `endif
  // pragma translate_on
endmodule

`include "axi/assign.svh"
`include "axi/typedef.svh"

module axi_mcast_xbar_intf
import cf_math_pkg::idx_width;
#(
  parameter int unsigned AXI_USER_WIDTH =  0,
  parameter axi_pkg::xbar_cfg_t Cfg     = '0,
  parameter bit ATOPS                   = 1'b1,
  parameter bit [Cfg.NoSlvPorts-1:0][Cfg.NoMstPorts-1:0] CONNECTIVITY = '1,
  parameter type rule_t                 = axi_pkg::xbar_rule_64_t
) (
  input  logic                        clk_i,
  input  logic                        rst_ni,
  input  logic                        test_i,
  AXI_BUS.Slave                       slv_ports [Cfg.NoSlvPorts-1:0],
  AXI_BUS.Master                      mst_ports [Cfg.NoMstPorts-1:0],
  input  rule_t [Cfg.NoAddrRules-1:0] addr_map_i
);

  localparam int unsigned AxiIdWidthMstPorts = Cfg.AxiIdWidthSlvPorts + $clog2(Cfg.NoSlvPorts);

  typedef logic [AxiIdWidthMstPorts     -1:0] id_mst_t;
  typedef logic [Cfg.AxiIdWidthSlvPorts -1:0] id_slv_t;
  typedef logic [Cfg.AxiAddrWidth       -1:0] addr_t;
  typedef logic [Cfg.AxiDataWidth       -1:0] data_t;
  typedef logic [Cfg.AxiDataWidth/8     -1:0] strb_t;
  typedef logic [AXI_USER_WIDTH         -1:0] user_t;
  // AW channel adds multicast mask to USER signals
  typedef struct packed {
    addr_t mcast;
  } aw_user_t;

  `AXI_TYPEDEF_AW_CHAN_T(mst_aw_chan_t, addr_t, id_mst_t, aw_user_t)
  `AXI_TYPEDEF_AW_CHAN_T(slv_aw_chan_t, addr_t, id_slv_t, aw_user_t)
  `AXI_TYPEDEF_W_CHAN_T(w_chan_t, data_t, strb_t, user_t)
  `AXI_TYPEDEF_B_CHAN_T(mst_b_chan_t, id_mst_t, user_t)
  `AXI_TYPEDEF_B_CHAN_T(slv_b_chan_t, id_slv_t, user_t)
  `AXI_TYPEDEF_AR_CHAN_T(mst_ar_chan_t, addr_t, id_mst_t, user_t)
  `AXI_TYPEDEF_AR_CHAN_T(slv_ar_chan_t, addr_t, id_slv_t, user_t)
  `AXI_TYPEDEF_R_CHAN_T(mst_r_chan_t, data_t, id_mst_t, user_t)
  `AXI_TYPEDEF_R_CHAN_T(slv_r_chan_t, data_t, id_slv_t, user_t)
  `AXI_TYPEDEF_REQ_T(mst_req_t, mst_aw_chan_t, w_chan_t, mst_ar_chan_t)
  `AXI_TYPEDEF_REQ_T(slv_req_t, slv_aw_chan_t, w_chan_t, slv_ar_chan_t)
  `AXI_TYPEDEF_RESP_T(mst_resp_t, mst_b_chan_t, mst_r_chan_t)
  `AXI_TYPEDEF_RESP_T(slv_resp_t, slv_b_chan_t, slv_r_chan_t)

  mst_req_t   [Cfg.NoMstPorts-1:0]  mst_reqs;
  mst_resp_t  [Cfg.NoMstPorts-1:0]  mst_resps;
  slv_req_t   [Cfg.NoSlvPorts-1:0]  slv_reqs;
  slv_resp_t  [Cfg.NoSlvPorts-1:0]  slv_resps;

  for (genvar i = 0; i < Cfg.NoMstPorts; i++) begin : gen_assign_mst
    `AXI_ASSIGN_FROM_REQ(mst_ports[i], mst_reqs[i])
    `AXI_ASSIGN_TO_RESP(mst_resps[i], mst_ports[i])
  end

  for (genvar i = 0; i < Cfg.NoSlvPorts; i++) begin : gen_assign_slv
    `AXI_ASSIGN_TO_REQ(slv_reqs[i], slv_ports[i])
    `AXI_ASSIGN_FROM_RESP(slv_ports[i], slv_resps[i])
  end

  axi_mcast_xbar #(
    .Cfg          (Cfg),
    .ATOPs        (ATOPS),
    .Connectivity (CONNECTIVITY),
    .slv_aw_chan_t(slv_aw_chan_t),
    .mst_aw_chan_t(mst_aw_chan_t),
    .w_chan_t     (w_chan_t),
    .slv_b_chan_t (slv_b_chan_t),
    .mst_b_chan_t (mst_b_chan_t),
    .slv_ar_chan_t(slv_ar_chan_t),
    .mst_ar_chan_t(mst_ar_chan_t),
    .slv_r_chan_t (slv_r_chan_t),
    .mst_r_chan_t (mst_r_chan_t),
    .slv_req_t    (slv_req_t),
    .slv_resp_t   (slv_resp_t),
    .mst_req_t    (mst_req_t),
    .mst_resp_t   (mst_resp_t),
    .rule_t       (rule_t)
  ) i_xbar (
    .clk_i,
    .rst_ni,
    .test_i,
    .slv_ports_req_i (slv_reqs),
    .slv_ports_resp_o(slv_resps),
    .mst_ports_req_o (mst_reqs),
    .mst_ports_resp_i(mst_resps),
    .addr_map_i
  );

endmodule
