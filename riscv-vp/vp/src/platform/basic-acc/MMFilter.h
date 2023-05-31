#ifndef MM_FILTER_H_
#define MM_FILTER_H_
#include <systemc>
#include <cmath>
#include <iomanip>
using namespace sc_core;

#include <tlm>
#include <tlm_utils/simple_target_socket.h>

#include "filter_def.h"

struct MMFilter : public sc_module {
  tlm_utils::simple_target_socket<MMFilter> tsock;

  sc_fifo<unsigned char> i_r;
  sc_fifo<unsigned char> i_g;
  sc_fifo<unsigned char> i_b;
  sc_fifo<int> o_result;

  SC_HAS_PROCESS(MMFilter);

  MMFilter(sc_module_name n): 
    sc_module(n), 
    tsock("t_skt"), 
    base_offset(0) 
  {
    tsock.register_b_transport(this, &MMFilter::blocking_transport);
    SC_THREAD(do_filter);
  }

  ~MMFilter() {
	}

  int val[MASK_N];
  unsigned int base_offset;
  sc_dt::sc_uint<12> pixel_r[9];
  sc_dt::sc_uint<12> pixel_g[9];
  sc_dt::sc_uint<12> pixel_b[9];
  unsigned char flattened_r;
  unsigned char flattened_g; 
  unsigned char flattened_b;
  sc_dt::sc_uint<12> sum_r;
	sc_dt::sc_uint<12> sum_g;
	sc_dt::sc_uint<12> sum_b;
  sc_dt::sc_uint<8> o_r;
  sc_dt::sc_uint<8> o_g;
  sc_dt::sc_uint<8> o_b;


  void do_filter(){
    // { wait(CLOCK_PERIOD, SC_NS); }
    while (true) {

      for (unsigned int v = 0; v < MASK_Y; ++v) {
        for (unsigned int u = 0; u < MASK_X; ++u) {
          pixel_r[v * 3 + u] = i_r.read();
          pixel_g[v * 3 + u] = i_g.read();
          pixel_b[v * 3 + u] = i_b.read();
        }
      }

      // 1. Applying "median filter" to each color channel
      int flattened_r[9],flattened_g[9],flattened_b[9];
      int k = 0;
      for (int i = 0; i < 3; i++) {
          for (int j = 0; j < 3; j++) {
              flattened_r[k] = pixel_r[i][j];
              flattened_g[k] = pixel_g[i][j];
              flattened_b[k] = pixel_b[i][j];
              k=k+1;
          }
      }
      std::sort(flattened_r, flattened_r + 9);
      std::sort(flattened_g, flattened_g + 9);
      std::sort(flattened_b, flattened_b + 9);

      unsigned char filtered_r = flattened_r[4];
      unsigned char filtered_g = flattened_g[4];
      unsigned char filtered_b = flattened_b[4];

      // 2. Applying "mean filter" to each color channel
      int sum_r = 0, sum_g = 0, sum_b = 0;

      for (unsigned int v = 0; v < MASK_Y; ++v) {
        for (unsigned int u = 0; u < MASK_X; ++u) {
          sum_r += filtered_r * mask[u][v];
          sum_g += filtered_g * mask[u][v];
          sum_b += filtered_b * mask[u][v];
        }
      }

      filtered_r = sum_r / 10;
      filtered_g = sum_g / 10;
      filtered_b = sum_b / 10;

      sc_dt::sc_uint<32> result = (0, filtered_b, filtered_g, filtered_r);
      o_result.write(result);


    }
  }

  void blocking_transport(tlm::tlm_generic_payload &payload, sc_core::sc_time &delay){
    wait(delay);
    // unsigned char *mask_ptr = payload.get_byte_enable_ptr();
    // auto len = payload.get_data_length();
    tlm::tlm_command cmd = payload.get_command();
    sc_dt::uint64 addr = payload.get_address();
    unsigned char *data_ptr = payload.get_data_ptr();

    addr -= base_offset;


    // cout << (int)data_ptr[0] << endl;
    // cout << (int)data_ptr[1] << endl;
    // cout << (int)data_ptr[2] << endl;
    word buffer;

    switch (cmd) {
      case tlm::TLM_READ_COMMAND:
        // cout << "READ" << endl;
        switch (addr) {
          case SOBEL_FILTER_RESULT_ADDR:
            buffer.uint = o_result.read();
            break;
          default:
            std::cerr << "READ Error! SobelFilter::blocking_transport: address 0x"
                      << std::setfill('0') << std::setw(8) << std::hex << addr
                      << std::dec << " is not valid" << std::endl;
          }
        data_ptr[0] = buffer.uc[0];
        data_ptr[1] = buffer.uc[1];
        data_ptr[2] = buffer.uc[2];
        data_ptr[3] = buffer.uc[3];
        break;
      case tlm::TLM_WRITE_COMMAND:
        // cout << "WRITE" << endl;
        switch (addr) {
          case SOBEL_FILTER_R_ADDR:
            i_r.write(data_ptr[0]);
            i_g.write(data_ptr[1]);
            i_b.write(data_ptr[2]);
            break;
          default:
            std::cerr << "WRITE Error! SobelFilter::blocking_transport: address 0x"
                      << std::setfill('0') << std::setw(8) << std::hex << addr
                      << std::dec << " is not valid" << std::endl;
        }
        break;
      case tlm::TLM_IGNORE_COMMAND:
        payload.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
        return;
      default:
        payload.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
        return;
      }
      payload.set_response_status(tlm::TLM_OK_RESPONSE); // Always OK
  }
};
#endif
