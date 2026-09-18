# FIFO Usage Guide

This covers the `fifo_sync` / `fifo_async` native interface, the difference
between a **first-word fall-through (FWFT)** FIFO and a **standard** FIFO, and
how an upper module should drive the ports.

See `docs/examples/fifo_adapter.sv` for compilable examples.

---

## 1. The native interface

| Direction | Signal | Meaning |
|---|---|---|
| Write | `i_wr_en` | Write strobe. Ignored when `o_full` is high. |
| Write | `i_wr_data` / `i_wr_user` | Payload and sideband. |
| Write | `o_full` / `o_almost_full` | No space / nearly no space. |
| Read | `i_rd_en` | Pop strobe. Consumes the head word when `o_empty` is low. |
| Read | `o_rd_data` / `o_rd_user` | Head word. **Valid whenever `o_empty` is low.** |
| Read | `o_empty` / `o_almost_empty` | No data / nearly no data. |
| Both | `o_level` | Occupancy. |

Write and read are independent ports: a `fifo_sync` can be written and read on
the same cycle. `fifo_async` splits them into `i_clk_w` (write, with `o_full`)
and `i_clk_r` (read, with `o_empty`, `o_level`), with FIFO depth `2**ADDR_WIDTH`.

---

## 2. FWFT vs standard FIFO

Both types use `o_empty` to say "a word is available". The difference is **when
`o_rd_data` is valid**.

### Standard (non-FWFT) FIFO

`o_rd_data` becomes valid **one cycle after** `i_rd_en` was high.

```
cycle          :  0     1     2     3
o_empty        :  0     0     0     1
i_rd_en        :  1     1     1     0
o_rd_data      :  X    D0    D1    D2     <- D0 appears one cycle after rd_en
```

A consumer must register the data and remember that it is one cycle behind the
`empty`/`rd_en` signalling. This is what makes non-FWFT interfaces awkward.

### First-word fall-through (this library)

The head word is presented **before** you ask, so `o_rd_data` is valid on the
same cycle that `o_empty` is low.

```
cycle          :  0     1     2     3
o_empty        :  0     0     0     1
o_rd_data      : D0    D1    D2     X     <- valid now, no request needed
i_rd_en        :  1     1     1     0     <- pops D0, D1, D2
```

Consequences:

* You may sample `o_rd_data` as soon as `o_empty` is low, then pulse `i_rd_en`
  to consume it (this is the safe "sample then pop" pattern).
* Alternatively, drive `i_rd_en = !o_empty & i_ready` and use `o_rd_data`
  directly (the valid/ready pattern).
* The FIFO sustains **one word per cycle** in steady state; `i_rd_en` on
  consecutive cycles pops consecutive words with no bubbles.
* After reset / first write there is a short fill latency before `o_empty`
  drops; do not assume `o_empty` is low the same cycle a word is written.

Why FWFT is preferred: it maps cleanly onto AXI-Stream / valid-ready streams
(`valid = ~empty`, `ready`/`rd_en` = pop) and lets you read one word per cycle
without a skid buffer.

---

## 3. How to drive the write side

Only ever assert `i_wr_en` when `o_full` is low.

```systemverilog
always_ff @(posedge i_clk or negedge i_rstn) begin
  if (!i_rstn) begin
    wr_en <= 1'b0;
  end else begin
    wr_en <= 1'b0;                 // one-cycle strobe by default
    if (!full) begin               // check full BEFORE writing
      wr_en   <= 1'b1;
      wr_data <= next_word;
    end
  end
end
```

For a valid/ready producer in front of the FIFO:

```systemverilog
assign o_s_ready = ~full;                  // can accept a beat now
assign wr_en     = i_s_valid && o_s_ready; // beat accepted this cycle
```

Back-to-back writes are allowed while `!full` (one per cycle). A write is
silently ignored on a cycle where `o_full` is high, so never rely on it being
accepted.

> `fifo_sync` also accepts a write in the same cycle a read is popping the last
> free slot (it deasserts `full` for you), but checking `!full` is still the
> portable rule.

---

## 4. How to drive the read side

### Pattern A - sample then pop (recommended for FSMs)

```systemverilog
// o_rd_data is already valid this cycle when !o_empty.
if (!empty) begin
  use_now(rd_data);            // e.g. register into your datapath
  rd_en <= 1'b1;               // pop the word at the next edge
end
```

### Pattern B - valid/ready passthrough

```systemverilog
assign o_m_valid = ~empty;                 // FWFT: no request needed
assign o_m_data  = rd_data;
assign rd_en     = o_m_valid & i_m_ready;  // pop exactly when accepted
```

`rd_en` must be a single-cycle strobe per word. Do **not** hold it high and
expect one pop - it pops one word per cycle it is high.

---

## 5. Packet and AXI-Stream layers

`fifo_packet` adds `i_wr_sop` / `i_wr_eop` and `o_rd_sop` / `o_rd_eop`; drive
the write side exactly as above and tag the first/last word of each packet. It
is a *streaming* FIFO: honour `o_full` (backpressure).

`axis_fifo` maps AXI-Stream directly:

```systemverilog
assign o_s_axis_tready = ~full;                                  // slave
assign wr_en           = i_s_axis_tvalid && o_s_axis_tready;
assign o_m_axis_tvalid = ~empty;                                 // master
assign rd_en           = o_m_axis_tvalid && i_m_axis_tready;
```

`axis_packet_fifo` is the same, but `o_m_axis_tvalid` is additionally gated on a
complete packet (tlast stored), i.e. store-and-forward. Because it backpressures
while full, a packet must fit inside the FIFO.

---

## 6. Example module

`docs/examples/fifo_adapter.sv` contains:

1. `fifo_adapter` - valid/ready stream in, valid/ready stream out, backed by a
   `fifo_sync`.
2. `fifo_producer_fsm` - an FSM that writes `N_WORDS` while respecting `i_full`.
3. `fifo_consumer_fsm` - FWFT read with an `i_ready` backpressure input.

### Typical producer/consumer wiring

```systemverilog
fifo_sync #(.DATA_WIDTH(32), .USER_WIDTH(1), .ADDR_WIDTH(4)) u_fifo (
  .i_clk(i_clk), .i_rstn(i_rstn),
  .i_wr_en(u_prod_wr_en), .i_wr_data(u_prod_wr_data), .i_wr_user(1'b0),
  .o_full(u_fifo_full), .o_almost_full(),
  .i_rd_en(u_cons_rd_en), .o_rd_data(u_fifo_rd_data), .o_rd_user(),
  .o_empty(u_fifo_empty), .o_almost_empty(), .o_level()
);

fifo_producer_fsm u_prod (
  .i_clk(i_clk), .i_rstn(i_rstn), .i_start(i_start), .o_busy(o_busy),
  .o_wr_en(u_prod_wr_en), .o_wr_data(u_prod_wr_data), .i_full(u_fifo_full)
);

fifo_consumer_fsm u_cons (
  .i_clk(i_clk), .i_rstn(i_rstn), .i_ready(i_ready),
  .i_empty(u_fifo_empty), .i_rd_data(u_fifo_rd_data),
  .o_rd_en(u_cons_rd_en), .o_valid(o_valid), .o_data(o_data)
);
```

---

## 7. Do / don't

| Do | Don't |
|---|---|
| Check `!o_full` before asserting `i_wr_en`. | Write while `o_full` and expect the data to be stored. |
| Sample `o_rd_data` when `!o_empty`. | Wait for a "data valid next cycle" like a standard FIFO. |
| Pulse `i_rd_en` one cycle per word. | Hold `i_rd_en` and expect a single pop. |
| Account for the startup fill latency. | Assume `o_empty` deasserts the same cycle as the first write. |
| Tie `i_clk_r`/`i_rstn_r` to the write clock for `CDC=0` wrappers. | Leave them floating. |
