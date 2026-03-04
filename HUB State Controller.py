import numpy as np
from gnuradio import gr
import pmt
import time

class pdu_req_reply(gr.basic_block):
    def __init__(self, initial_free_channels=[1, 2, 3, 4, 5], assignment_cooldown=5.0):
        gr.basic_block.__init__(
            self,
            name="HUB_Controller",
            in_sig=None,
            out_sig=None
        )
        self.message_port_register_in(pmt.intern("pdu_in"))
        self.message_port_register_out(pmt.intern("pdu_out"))
        self.set_msg_handler(pmt.intern("pdu_in"), self.handle_msg)
        
        self.free_channels = list(initial_free_channels)
        
        # Dizionario per memorizzare assegnazioni attive: {ID_SENSORE: (CANALE, TIMESTAMP)}
        self.active_assignments = {}
        self.cooldown = assignment_cooldown

    def handle_msg(self, msg):
        data_pmt = pmt.cdr(msg)
        if pmt.is_u8vector(data_pmt):
            payload_bytes = pmt.u8vector_elements(data_pmt)
            if len(payload_bytes) > 24:
                try:
                    # Decodifica latin-1 per compatibilità con il range 0-255
                    decoded_str = bytes(payload_bytes[24:]).decode('latin-1').strip()
                    
                    if decoded_str.startswith("REQ"):
                        print(f"[1] HUB ricevuto: {decoded_str}")
                        
                        # Estrazione ID (es: da "REQ_1" estrae "1")
                        parts = decoded_str.split('_')
                        sensor_id = parts[1] if len(parts) > 1 else "0"
                        
                        now = time.time()
                        
                        # LOGICA ANTI-BURN: Se il sensore ha già un canale assegnato recentemente,
                        # riproponiamo lo stesso senza estrarne uno nuovo dalla lista.
                        if sensor_id in self.active_assignments:
                            assigned_ch, last_time = self.active_assignments[sensor_id]
                            if (now - last_time) < self.cooldown:
                                reply_str = f"REQ{assigned_ch}_{sensor_id}"
                                print(f"[RE-TX] HUB ripropone {reply_str} per ID {sensor_id}")
                                self._send_reply(reply_str)
                                return

                        # NUOVA ASSEGNAZIONE
                        if self.free_channels:
                            assigned_ch = self.free_channels.pop(0)
                            self.active_assignments[sensor_id] = (assigned_ch, now)
                            
                            reply_str = f"REQ{assigned_ch}_{sensor_id}"
                            print(f"[2] HUB assegnazione: {reply_str}")
                            self._send_reply(reply_str)
                        else:
                            print(f"[!] HUB canali occupati (richiesta da ID {sensor_id})")
                            
                except Exception as e:
                    print(f"Errore HUB: {e}")

    def _send_reply(self, reply_str):
        """Helper per inviare la PDU di risposta"""
        reply_payload = reply_str.encode('latin-1')
        meta = pmt.make_dict()
        data = pmt.init_u8vector(len(reply_payload), list(reply_payload))
        self.message_port_pub(pmt.intern("pdu_out"), pmt.cons(meta, data))
