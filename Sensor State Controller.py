import threading
import time
import pmt
import random
import numpy as np
from gnuradio import gr

class sensor_controller(gr.sync_block):
    def __init__(self, sensor_id=1, max_backoff_ms=500, max_retries=5, retry_delay_ms=2000, max_backoff_attempts=10):
        gr.sync_block.__init__(
            self, 
            name='controllore sensore', 
            in_sig=[np.float32], 
            out_sig=None
        )
        
        self.message_port_register_in(pmt.intern("pdu_in"))
        self.message_port_register_in(pmt.intern("payload_in"))
        self.message_port_register_out(pmt.intern("pdu_out"))
        self.message_port_register_out(pmt.intern("freq_out"))
        
        self.set_msg_handler(pmt.intern("pdu_in"), self.handle_pdu)
        self.set_msg_handler(pmt.intern("payload_in"), self.handle_payload)
        
        self.sensor_id = sensor_id
        self.max_backoff_ms = max_backoff_ms
        self.max_retries = max_retries
        self.retry_delay_ms = retry_delay_ms
        self.max_backoff_attempts = max_backoff_attempts
        
        self.settling_time = 0.05
        self.guard_interval = 0.5
        
        self.offsets = [-4.167e6, -2.500e6, -0.833e6, 0.833e6, 2.500e6, 4.167e6]
        self.current_channel = 0
        self.current_state = "REQ"
        self.req_attempts = 0
        self.backoff_attempts = 0
        
        self.channel_is_free = True 
        self.last_sample_time = time.time()
        self.running = False
        self.thread = None

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self.run_loop)
        self.thread.start()
        return True

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join()
        return True

    def work(self, input_items, output_items):
        if len(input_items[0]) > 0:
            valore_grezzo = input_items[0][-1]
            self.channel_is_free = (valore_grezzo < 0.5)
            self.last_sample_time = time.time() 
        return len(input_items[0])

    def handle_payload(self, msg):
        # Fase di negoziazione: blocco trasmissione dati se non in stato SIG
        if self.current_state != "SIG":
            return

        if pmt.is_u8vector(msg):
            raw_payload = list(pmt.u8vector_elements(msg))
            offset = self.offsets[self.current_channel]
            print(f"[4] SENSORE {self.sensor_id} f {offset} TX raw {raw_payload}")
            
            meta = pmt.make_dict()
            data = pmt.init_u8vector(len(raw_payload), raw_payload)
            self.message_port_pub(pmt.intern("pdu_out"), pmt.cons(meta, data))

    def handle_pdu(self, msg):
        # Pulizia REQ: ignora risposte HUB se la negoziazione è già conclusa
        if self.current_state != "REQ":
            return

        data_pmt = pmt.cdr(msg)
        if pmt.is_u8vector(data_pmt):
            payload_bytes = pmt.u8vector_elements(data_pmt)
            if len(payload_bytes) > 24:
                try:
                    decoded_str = bytes(payload_bytes[24:]).decode('latin-1').strip()
                    
                    if decoded_str.startswith("REQ") and f"_{self.sensor_id}" in decoded_str:
                        ch_part = decoded_str.split('_')[0]
                        ch_str = "".join(filter(str.isdigit, ch_part))
                        
                        if ch_str:
                            ch_index = int(ch_str)
                            if 0 <= ch_index < len(self.offsets):
                                current_freq = self.offsets[self.current_channel]
                                freq_target = self.offsets[ch_index]
                                
                                print(f"[2] SENSORE {self.sensor_id} RX VALIDO: {decoded_str}")
                                print(f"[3] SENSORE {self.sensor_id} SALTO su f-{freq_target}")
                                
                                # Transizione di stato atomica
                                self.current_channel = ch_index
                                self.current_state = "SIG"
                                time.sleep(self.guard_interval)
                except Exception as e:
                    print(f"Errore handle_pdu: {e}")

    def run_loop(self):
        while self.running:
            offset = self.offsets[self.current_channel]
            msg_freq = pmt.cons(pmt.intern("freq"), pmt.from_double(offset))
            self.message_port_pub(pmt.intern("freq_out"), msg_freq)
            
            time.sleep(self.settling_time)

            if (time.time() - self.last_sample_time) > 0.1:
                self.channel_is_free = True

            if self.current_state == "REQ":
                if self.req_attempts >= self.max_retries:
                    print(f"[!] SENSORE {self.sensor_id} f {offset} ERRORE - REQ max")
                    self.current_state = "ERROR"
                    continue

                if self.channel_is_free:
                    self.req_attempts += 1
                    self.backoff_attempts = 0 
                    print(f"[1] SENSORE {self.sensor_id} f {offset} CCA libero - TX REQ {self.req_attempts}/{self.max_retries}")
                    
                    req_msg = f"REQ_{self.sensor_id}"
                    raw_payload = list(req_msg.encode('utf-8'))
                    
                    meta = pmt.make_dict()
                    data = pmt.init_u8vector(len(raw_payload), raw_payload)
                    self.message_port_pub(pmt.intern("pdu_out"), pmt.cons(meta, data))
                    time.sleep(self.retry_delay_ms / 1000.0)
                else:
                    if self.backoff_attempts >= self.max_backoff_attempts:
                        self.current_state = "ERROR"
                        continue
                    self.backoff_attempts += 1
                    backoff = random.uniform(0, self.max_backoff_ms)
                    print(f"[0] SENSORE {self.sensor_id} f {offset} CCA occupato - Backoff {self.backoff_attempts}: {backoff:.0f} ms")
                    time.sleep(backoff / 1000.0)
            
            elif self.current_state == "SIG":
                time.sleep(0.1)
            elif self.current_state == "ERROR":
                time.sleep(1.0)
