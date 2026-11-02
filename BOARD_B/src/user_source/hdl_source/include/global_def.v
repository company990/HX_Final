


`define   DATA_WIDTH                        32
`define   ADDR_WIDTH                        21
`define   DM_WIDTH                          4

`define   ROW_WIDTH                        11
`define   BA_WIDTH                        2

// Parenthesize each macro: Verilog substitutes tokens, not evaluated values.
// Without parentheses the old interval expanded to 64000000/1000000000/
// 50000000/2048 == 0, suppressing all periodic refresh requests.
`define SDR_CLK_PERIOD (1000000000 / 50000000)
// Refresh all 2048 rows within 32ms (margin against the 64ms requirement
// and deferred requests during memory bursts). Counter includes cycle zero.
`define SELF_REFRESH_INTERVAL ((32000000 / `SDR_CLK_PERIOD / (1 << `ROW_WIDTH)) - 1)

