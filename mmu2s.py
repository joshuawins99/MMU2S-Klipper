#!/usr/bin/env python3
import serial
import time

MMU_register_table = {
    "pulley_slow_feedrate"  : 0x14,
    "pulley_load_feedrate"  : 0x11,
    "FINDA_State"           : 0x08,
    "FSensor_State"         : 0x09,
    "Filament_State"        : 0x07,
    "Set_Get_Selector_slot" : 0x1b,
    "Set_Get_Idler_slot"    : 0x1c,
    "bowden_length"         : 0x22,
    "cut_length"            : 0x23
}

MMU_status_table = {
    'P': 'Processing',
    'E': 'Error',
    'F': 'Finished',
    'A': 'Accepted',
    'R': 'Rejected',
    'B': 'Button',
}

MMU_error_table = {
    0x0000: "RUNNING",
    0x0001: "OK",

    # TMC bit masks
    0x0040: "TMC_PULLEY_BIT",
    0x0080: "TMC_SELECTOR_BIT",
    0x0100: "TMC_IDLER_BIT",

    # Filament / sensor errors
    0x8001: "FINDA_DIDNT_SWITCH_ON",
    0x8002: "FINDA_DIDNT_SWITCH_OFF",
    0x8003: "FSENSOR_DIDNT_SWITCH_ON",
    0x8004: "FSENSOR_DIDNT_SWITCH_OFF",
    0x8005: "FILAMENT_ALREADY_LOADED",
    0x8006: "INVALID_TOOL",

    # Homing errors
    0x8007: "HOMING_FAILED",
    0x8087: "HOMING_SELECTOR_FAILED",
    0x8107: "HOMING_IDLER_FAILED",
    0x8047: "STALLED_PULLEY",

    0x8008: "FINDA_VS_EEPROM_DISREPANCY",
    0x8009: "FSENSOR_TOO_EARLY",
    0x800A: "FINDA_FLICKERS",

    # Move errors
    0x800B: "MOVE_FAILED",
    0x808B: "MOVE_SELECTOR_FAILED",
    0x810B: "MOVE_IDLER_FAILED",
    0x804B: "MOVE_PULLEY_FAILED",

    0x800C: "FILAMENT_EJECTED",
    0x800D: "MCU_UNDERVOLTAGE_VCC",

    0x8029: "FILAMENT_CHANGE",
    0x802A: "LOAD_TO_EXTRUDER_FAILED",
    0x802B: "QUEUE_FULL",
    0x802C: "VERSION_MISMATCH",
    0x802D: "PROTOCOL_ERROR",
    0x802E: "MMU_NOT_RESPONDING",
    0x802F: "INTERNAL",

    # TMC driver errors
    0x8200: "TMC_IOIN_MISMATCH",
    0x8400: "TMC_RESET",
    0x8800: "TMC_UNDERVOLTAGE_ON_CHARGE_PUMP",
    0x9000: "TMC_SHORT_TO_GROUND",
    0xA000: "TMC_OVER_TEMPERATURE_WARN",
    0xC000: "TMC_OVER_TEMPERATURE_ERROR",
    0xC200: "MMU_SOLDERING_NEEDS_ATTENTION",
}

# ------------------------------------------------------------
# MMU2S CRC8
# ------------------------------------------------------------
def crc8_updateCX(crc, byte):
    crc ^= byte
    for _ in range(8):
        if crc & 0x80:
            crc = ((crc << 1) ^ 0x07) & 0xFF
        else:
            crc = (crc << 1) & 0xFF
    return crc

def crc8_updateW(crc, word):
    hi = (word >> 8) & 0xFF
    lo = word & 0xFF
    crc = crc8_updateCX(crc, hi)
    crc = crc8_updateCX(crc, lo)
    return crc

def compute_crc_request(code, value, value2=0):
    crc = 0
    crc = crc8_updateCX(crc, code)
    crc = crc8_updateCX(crc, value & 0xFF)
    crc = crc8_updateCX(crc, value2 & 0xFF)        # low byte first
    crc = crc8_updateCX(crc, (value2 >> 8) & 0xFF) # high byte second
    return crc

# ------------------------------------------------------------
# Hex formatting
# ------------------------------------------------------------
def uint8_to_hex(v):
    if v == 0:
        return "0"
    s = ""
    if v >> 4:
        s += format((v >> 4) & 0xF, "x")
    s += format(v & 0xF, "x")
    return s

def uint16_to_hex(v):
    if v == 0:
        return "0"
    s = ""
    started = False
    for shift in (12, 8, 4, 0):
        nib = (v >> shift) & 0xF
        if nib != 0 or started:
            s += format(nib, "x")
            started = True
    return s

# ------------------------------------------------------------
# Request encoding
# ------------------------------------------------------------
def encode_request(code_char, value):
    """
    Generic request:
      <Code><HexValue>*<CRC>\n
    Example:
      T1*ab\n
      Q0*cd\n
    """
    code = ord(code_char)
    value_hex = uint8_to_hex(value)
    body = f"{code_char}{value_hex}"
    crc = compute_crc_request(code, value)
    crc_hex = uint8_to_hex(crc)
    return f"{body}*{crc_hex}\n"


def encode_write_request(address, value):
    addr_hex = uint8_to_hex(address)
    val_hex = uint16_to_hex(value)
    crc = compute_crc_request(ord('W'), address, value)
    crc_hex = uint8_to_hex(crc)
    return f"W{addr_hex} {val_hex}*{crc_hex}\n"

def encode_read_request(address: int) -> str:
    """
    Read request:
      R<addr>*<CRC>\n
    """
    addr_hex = uint8_to_hex(address)
    body = f"R{addr_hex}"
    crc = compute_crc_request(ord('R'), address, 0)
    crc_hex = uint8_to_hex(crc)
    return f"{body}*{crc_hex}\n"

# ------------------------------------------------------------
# Response decoding
# ------------------------------------------------------------
class MMUResponse:
    def __init__(self, req_code, req_value, param_code, param_value):
        self.req_code = req_code       # 'T', 'L', 'Q', etc.
        self.req_value = req_value     # hex value after request code
        self.param_code = param_code   # 'P', 'E', 'F', 'A', 'R', 'B'
        self.param_value = param_value # hex value after param code

    def __repr__(self):
        return f"<MMUResponse {self.req_code}{self.req_value:x} {self.param_code}{self.param_value:x}>"

def decode_response(line):
    line = line.strip()

    if not line or "*" not in line:
        return None

    try:
        body, crc_hex = line.split("*", 1)
        int(crc_hex, 16)      # Just validate for now
    except (ValueError, IndexError):
        return None

    if " " not in body:
        return None

    try:
        req_part, param_part = body.split(" ", 1)

        if not req_part or not param_part:
            return None

        req_code = req_part[0]
        req_val_str = req_part[1:]
        req_value = int(req_val_str, 16) if req_val_str else 0

        param_code = param_part[0]
        param_val_str = param_part[1:]
        param_value = int(param_val_str, 16) if param_val_str else 0

    except (ValueError, IndexError):
        return None

    return MMUResponse(req_code, req_value, param_code, param_value)

def translate_status(resp):
    # Only Q0 responses carry status
    if resp.req_code == None:
        return None

    status_name = MMU_status_table.get(resp.param_code, 'Unknown')

    return {
        'status': status_name,
        'code': resp.param_code,
        'value': resp.param_value,
    }

# ------------------------------------------------------------
# MMU2S UART wrapper
# ------------------------------------------------------------
class MMU2S:
    def __init__(self, port="/dev/ttyS0", baud=115200, timeout=1.0):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(0.05)

    def flush_input(self):
        # Clear any stale bytes before sending a new command
        self.ser.reset_input_buffer()

    def send_frame(self, frame: str):
        self.flush_input()
        self.ser.write(frame.encode("ascii"))
        self.ser.flush()
        time.sleep(0.02)

    def read_line(self) -> str:
        return self.ser.readline().decode("ascii", errors="ignore")

    def request(self, code_char: str, value: int = 0, retries: int = 3, wait_for_response: bool = True) -> MMUResponse | None:
        frame = encode_request(code_char, value)

        if wait_for_response == True:
            for attempt in range(retries):
                self.send_frame(frame)
                response = self._wait_response()

                if isinstance(response, MMUResponse) or code_char == 'X':
                    return response

                # short delay before retry
                time.sleep(0.05)
        else:
            self.send_frame(frame)
        return None

    def write_register(self, addr: int, value: int) -> MMUResponse | None:
        frame = encode_write_request(addr, value)
        self.send_frame(frame)
        return self._wait_response()

    def read_register(self, addr: int) -> int | None:
        frame = encode_read_request(addr)
        self.send_frame(frame)
        resp = self._wait_response()
        if resp is None:
            return None
        if resp.param_code != 'A':
            return None
        return resp.param_value

    def read_register_retry(self, addr: int, attempts=3):
        for _ in range(attempts):
            val = self.read_register(addr)
            if val is not None:
                return val
            time.sleep(0.05)
        return None

    def _wait_response(self, timeout: float = 2.0) -> MMUResponse | None:
        end = time.time() + timeout
        last_line = ""
        while time.time() < end:
            line = self.read_line()
            if not line:
                continue
            last_line = line.strip()
            rsp = decode_response(line)
            if rsp is not None:
                return rsp
        return None

    def get_status(self):
        resp = self.request('Q', 0)
        if not isinstance(resp, MMUResponse):
            return None
        return translate_status(resp)

class MMUFindaSensor:
    def __init__(self, printer, mmu, poll_rate):
        self.printer = printer
        self.mmu = mmu
        self.state = False
        self.poll_rate = poll_rate
        self.enabled = False

        self.reactor = printer.get_reactor()
        self.reactor.register_timer(
            self._poll,
            self.reactor.monotonic() + 0.002
        )

    def _poll(self, eventtime):
        status = self.mmu.get_status()
        if status['status'] != 'Finished' or status['value'] != 0:
            return eventtime + self.poll_rate
        self.state = (self.mmu.read_register_retry(MMU_register_table["FINDA_State"]) != 0)
        return eventtime + self.poll_rate

    def get_status(self, eventtime):
        return {
            "enabled": self.enabled,
            "filament_detected": self.state
        }

class MMU2S_Klipper:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()

        # Defined variables in klipper instantiation
        port                      = config.get('serial')
        baud                      = config.getint('baud', 115200)
        timeout                   = config.getfloat('timeout', 1.0)
        self.load_total_distance  = config.getfloat('load_total_distance', 250.0) # Used as a timeout for filament to load between the FINDA and Filament Extruder Sensor
        self.load_step            = config.getfloat('load_step', 5.0) # Used as a granularity amount for gcode commands send to extruder motor while waiting for filament to arrive
        self.load_extra_pull      = config.getfloat('load_extra_pull', 5.0) # Extra amount to pull filament in after filament detected from a load
        self.load_extra_retract   = config.getfloat('load_extra_retract', -30.0) # Amount to retract filament after extra pull. If extra_pull is 0 then this is ignored. 
        self.load_slow_feedrate   = config.getint('load_slow_feedrate', 0) # Feedrate of the extruder motor to pull filament in. 0 reads MMU register to match it. In mm/s
        self.load_feedrate_factor = config.getfloat('load_feedrate_factor', 1) # Multiplier for during MMU to extruder handoff
        self.unload_retract_speed = config.getfloat('unload_retract_speed', 20.0) # Initial extruder retraction speed on an unload from extruder
        self.unload_step          = config.getfloat('unload_step', 2.0) # Used as a granularity amount for gcode commands send to extruder motor while waiting for filament sensor to deactivate
        self.unload_max_retract   = config.getfloat('unload_max_retract', 30.0) # Used as a timeout for filament to unload from extruder
        self.FINDA_poll_rate      = config.getfloat('FINDA_poll_rate', 1) # Polling rate of FINDA probe
        self.mmu_slow_feedrate    = config.getint('mmu_slow_feedrate', 0) # Allows for writing MMU register to change slow_feedrate. 0 means use default
        self.mmu_load_feedrate    = config.getint('mmu_load_feedrate', 0) # Allows for writing MMU register to change load_feedrate. 0 means use default
        self.mmu_bowden_length    = config.getint('mmu_bowden_length', 0) # Allows for writing MMU register to change bowden_length. 0 means use default
        self.mmu_cut_length       = config.getint('mmu_cut_length', 0) # Allows for writing MMU register to change cut_length. 0 means use default
        self.mmu_restart          = config.getboolean('mmu_restart', True) # Toggles MMU restarting when Klipper initializes

        # Instantiate MMU2S driver
        self.mmu = MMU2S(port=port, baud=baud, timeout=timeout)

        if self.mmu_restart == True:
            self.mmu.request('X', 0)
        time.sleep(1)
        # Hack to initialize comms
        self.mmu.request('Q', 0)
        self.mmu.request('Q', 0)
        self.mmu.request('Q', 0)

        self.changing_spools = False
        self.fail_counter = 0
        self.max_fail_count = 3

        if self.mmu_slow_feedrate != 0: # Write register to update slow feedrate of MMU
            self.mmu.write_register(MMU_register_table['pulley_slow_feedrate'], self.mmu_slow_feedrate)

        if self.mmu_load_feedrate != 0: # Write register to update load feedrate of MMU
            self.mmu.write_register(MMU_register_table['pulley_load_feedrate'], self.mmu_load_feedrate)

        if self.mmu_bowden_length != 0: # Write register to update bowden length of MMU
            self.mmu.write_register(MMU_register_table['bowden_length'], self.mmu_bowden_length)

        if self.mmu_cut_length != 0: # Write register to update cut length of MMU
            self.mmu.write_register(MMU_register_table['cut_length'], self.mmu_cut_length)

        gcode = self.printer.lookup_object('gcode')

        gcode.register_command('MMU_UNLOAD', self.cmd_MMU_UNLOAD)
        gcode.register_command('MMU_PRELOAD', self.cmd_MMU_PRELOAD)
        gcode.register_command('MMU_STATUS', self.cmd_MMU_STATUS)
        gcode.register_command('MMU_LOAD', self.cmd_MMU_LOAD)
        gcode.register_command('MMU_CUT', self.cmd_MMU_CUT)
        gcode.register_command('MMU_REG_READ', self.cmd_MMU_REG_READ)
        gcode.register_command('MMU_RESET', self.cmd_MMU_RESET)
        gcode.register_command('MMU_SET_SPOOLJOIN', self.cmd_MMU_SET_SPOOLJOIN)

        self.printer.add_object("filament_switch_sensor MMU_Finda", MMUFindaSensor(self.printer, self.mmu, self.FINDA_poll_rate))

        self.printer.register_event_handler("klippy:ready", self._spooljoin_handler)
        self.printer.register_event_handler("klippy:ready", self._initialize_vars)

        #reactor = self.printer.get_reactor()
        #self._poll_timer = reactor.register_timer(self._poll_mmu)

        #self.printer.register_event_handler("klippy:ready", self.handle_ready)
        #self.handle_ready()

    def _initialize_vars(self):
        self.reactor.register_callback(self._run_initialization_gcode)

    def _run_initialization_gcode(self, eventtime):
        gcode = self.printer.lookup_object("gcode")
        gcode.run_script_from_command(f"SAVE_VARIABLE VARIABLE=mmu_spooljoin_enable VALUE=0")

    def _spooljoin_handler(self):
        self.reactor.register_timer(self.spool_join, self.reactor.monotonic() + 0.5)

    def handle_ready(self):
        reactor = self.printer.get_reactor()
        reactor.update_timer(self._poll_timer, reactor.NOW)

    def _poll_mmu(self, eventtime):
        try:
            self.mmu.get_status()
        except Exception as e:
            self.gcode.respond_info(f"MMU: Timed Out!")

        # Poll again in 1 second
        return eventtime + 1

    def _pause_print(self):
        reactor = self.printer.get_reactor()
        gcode = self.printer.lookup_object("gcode")
        pause_resume = self.printer.lookup_object("pause_resume")
        print_stats = self.printer.lookup_object("print_stats")
        is_printing = print_stats.get_status(reactor.monotonic())["state"] == "printing"
        if is_printing and not pause_resume.is_paused:
            gcode.run_script_from_command("PAUSE")

    def spool_join(self, eventtime):
        gcode = self.printer.lookup_object("gcode")
        sv = self.printer.lookup_object("save_variables")
        reactor = self.printer.get_reactor()
        print_stats = self.printer.lookup_object("print_stats")
        spool_join_enable = sv.allVariables.get("mmu_spooljoin_enable", 0)
        sensor = self.printer.lookup_object("filament_switch_sensor fsensor")
        self.changing_spools = False
        next_spool = -1

        if sensor.get_status(reactor.monotonic()).get("filament_detected", False): # Optical sensor is inverted logic
            return eventtime + 0.5

        if spool_join_enable == 0:
            return eventtime + 0.5
        
        spool_order = sv.allVariables.get("mmu_spooljoin", [])
        current_slot = sv.allVariables.get("mmu_loaded_slot", -1)
        is_printing = print_stats.get_status(reactor.monotonic())["state"] == "printing"

        if current_slot == -1 and is_printing: # Shouldn't hit this case
            gcode.respond_info("MMU: ERROR. Unexpected case hit in spool join function!")
            return eventtime + 0.5

        if is_printing == False: # Not printing
            return eventtime + 0.5

        current_slot += 1 # Switch to 1 based counting

        for idx, slot in enumerate(spool_order):
            if slot == current_slot:
                if (idx < len(spool_order)-1):
                    next_spool = spool_order[idx+1]
                    spool_order[idx] = -1 # Clear Slot
                    self.save_spooljoin(spool_order) # Save updated spool order
                    self.changing_spools = True
                    break
                else:
                    gcode.run_script_from_command(f"SAVE_VARIABLE VARIABLE=mmu_spooljoin_enable VALUE=0")

        if self.changing_spools == True:
            gcode.respond_info(f"MMU: Spool {current_slot} empty. Switching to {next_spool}")
            gcode.run_script_from_command(f"SAVE_VARIABLE VARIABLE=mmu_spooljoin_next_spool VALUE={next_spool-1}") # MMU_LOAD SLOT is 0 based
            gcode.run_script_from_command("MMU_SPOOLJOIN_SEQUENCE")
        
        return eventtime + 0.5

    def cmd_MMU_LOAD(self, gcmd):
        slot = gcmd.get_int('SLOT')
        gcode = self.printer.lookup_object("gcode")
        reactor = self.printer.get_reactor()
        save_variables = self.printer.lookup_object("save_variables")
        just_unloaded = False

        while True:
            status = self.mmu.get_status()
            if status['status'] == 'Finished' or status['value'] == 0:
                break
            if status['status'] == 'Error':
                gcmd.respond_info(f"MMU: Error -> {status['value']}")
                gcode.run_script_from_command(f"SAVE_VARIABLE VARIABLE=mmu_loaded_slot VALUE=-1")
                return
            reactor.pause(reactor.monotonic() + 0.002)

        finda_state = self.mmu.read_register_retry(MMU_register_table["FINDA_State"])

        gcode.run_script_from_command(f"SAVE_VARIABLE VARIABLE=mmu_spooljoin_enable VALUE=1")

        loaded_slot = save_variables.allVariables.get("mmu_loaded_slot", -1)
        if loaded_slot == slot:
            return

        gcode.run_script_from_command(f"SAVE_VARIABLE VARIABLE=mmu_changing_slot VALUE={slot}")
        gcode.run_script_from_command(f"SAVE_VARIABLE VARIABLE=mmu_previous_slot VALUE={loaded_slot}")

        if loaded_slot != slot and finda_state != 0:
            self.cmd_MMU_UNLOAD(gcmd)
            just_unloaded = True
        
        loaded_slot = save_variables.allVariables.get("mmu_loaded_slot", -1)
        if loaded_slot == -1 and finda_state != 0:
            gcmd.respond_info(f"MMU: Loaded Slot Unknown. Preventing a potential unintended load. Please reset first")
            return

        # Wait for idle
        while True:
            status = self.mmu.get_status()
            if status['status'] == 'Finished' or status['value'] == 0:
                break
            if status['status'] == 'Error':
                error_name = MMU_error_table.get(status['value'], f"UNKNOWN_ERROR_0x{status['value']:04X}")
                gcmd.respond_info(f"MMU: Error -> {error_name}")
                self._pause_print()
                return
            reactor.pause(reactor.monotonic() + 0.002)

        # Read the new state
        if just_unloaded:
            finda_state = self.mmu.read_register_retry(MMU_register_table["FINDA_State"])

            if finda_state != 0: 
                gcmd.respond_info(f"MMU: FINDA Probe activated after unload. Aborting loading of filament") 
                return

        self.gcmd = gcmd
        gcmd.respond_info(f"MMU: Loading Slot {slot}")
        
        while self.fail_counter < self.max_fail_count:
            pass_fail = self._start_loading(gcmd, slot)
            if pass_fail == False and self.fail_counter == 2:
                gcode.run_script_from_command(f"MMU_CUT SLOT={slot}")
            if pass_fail == True:
                break

    def _filament_sensor_check(self, eventtime):
        reactor = self.printer.get_reactor()
        sensor = self.printer.lookup_object("filament_switch_sensor fsensor")

        self._filament_detected = sensor.get_status(eventtime).get("filament_detected", False)

        if self._filament_detected:
            self.mmu.request('f', 1, wait_for_response=False)
            self._filament_sensor_triggered = True

        return eventtime + 0.001

    def _check_filament_grab(self, gcmd):
        reactor = self.printer.get_reactor()
        gcode = self.printer.lookup_object("gcode")
        sensor = self.printer.lookup_object("filament_switch_sensor fsensor")

        initial = sensor.get_status(reactor.monotonic()).get("filament_detected", False)
        self._filament_detected = initial

        # Forward
        gcode.run_script_from_command("G1 E20 F2000")
        gcode.run_script_from_command("M400")
        forward = self._filament_detected

        # Reverse
        gcode.run_script_from_command("G1 E-10 F2000")
        gcode.run_script_from_command("M400")
        reverse = self._filament_detected

        gcmd.respond_info(f"MMU: Grab Test -> initial={initial}, forward={forward}, reverse={reverse}")

        # Filament should have always triggered sensor
        return (initial and forward and reverse)

    def _start_loading(self, gcmd, slot):
        gcode = self.printer.lookup_object("gcode")
        sensor = self.printer.lookup_object("filament_switch_sensor fsensor")
        reactor = self.printer.get_reactor()

        if self.load_slow_feedrate == 0:
            slow_feedrate = float(self.mmu.read_register_retry(MMU_register_table["pulley_slow_feedrate"]))
        else:
            slow_feedrate = float(self.load_slow_feedrate)

        # G1 uses mm/min
        feedrate = slow_feedrate * 60.0
        gcmd.respond_info(f"MMU: Loading with feedrate {slow_feedrate} mm/s ({feedrate} mm/min)")

        self.mmu.request('T', slot)
        self.mmu.request('f', 0)

        # Save current extrusion mode
        gcode.run_script_from_command("SAVE_GCODE_STATE NAME=MMU_LOAD")
        gcode.run_script_from_command("M83")      # Relative extrusion

        moved = 0.0
        loading_failed = False
        self._filament_sensor_triggered = False
        self._filament_sensor_timer = None
        self._filament_sensor_timer = reactor.register_timer(self._filament_sensor_check, reactor.monotonic())
        try:
            while moved < self.load_total_distance:
                if self._filament_sensor_triggered == True:
                    if self._filament_sensor_timer is not None:
                        reactor.update_timer(self._filament_sensor_timer, reactor.NEVER)
                        self._filament_sensor_timer = None
                    gcode.run_script_from_command(f"G4 P1500")
                    while True: # Wait until completed to finish loading
                        status = self.mmu.get_status()
                        if status['status'] == 'Finished' or status['value'] == 0:
                            break
                        if status['status'] == 'Error':
                            error_name = MMU_error_table.get(status['value'], f"UNKNOWN_ERROR_0x{status['value']:04X}")
                            gcmd.respond_info(f"MMU: Loading Error -> {error_name}")
                            self._pause_print()
                            break
                        reactor.pause(reactor.monotonic() + 0.002)
                    gcode.run_script_from_command(f"G1 E1 F{feedrate:.0f}")
                    gcode.run_script_from_command(f"M400")
                    if not self._check_filament_grab(gcmd):
                        gcmd.respond_info(f"MMU: Loading Error -> Didnt Pass Grab Test")
                        loading_failed = True
                        break
                    if self.load_extra_pull != 0:
                        gcode.run_script_from_command(f"G1 E{self.load_extra_pull:.3f} F{feedrate:.0f}")
                        gcode.run_script_from_command("M400")
                    if self.load_extra_retract != 0:
                        gcode.run_script_from_command(f"G1 E{self.load_extra_retract:.3f} F{feedrate:.0f}")
                        gcode.run_script_from_command("M400")
                    break
                gcode.run_script_from_command(f"G1 E{self.load_step:.3f} F{feedrate*self.load_feedrate_factor:.0f}")
                #gcode.run_script_from_command("M400")
                moved += self.load_step
            else:
                gcmd.respond_info(f"MMU: Filament not detected after {self.load_total_distance} mm.")
                #self._pause_print()
                # Tell MMU filament sensor activated to stop motor grinding
                self.mmu.request('f', 1)
                while True:
                    status = self.mmu.get_status()
                    if status['status'] == 'Finished' or status['value'] == 0:
                        break
                    if status['status'] == 'Error':
                        error_name = MMU_error_table.get(status['value'], f"UNKNOWN_ERROR_0x{status['value']:04X}")
                        gcmd.respond_info(f"MMU: Error -> {error_name}")
                        break
                    reactor.pause(reactor.monotonic() + 0.002)
                # Move selector to park slot
                #self.mmu.write_register(MMU_register_table['Set_Get_Selector_slot'], 5)
                #gcmd.respond_info(f"MMU: Print Paused due to loading issue with slot {slot}")
                loading_failed = True

            if loading_failed == True:
                self.fail_counter += 1
                gcode.run_script_from_command(f"MMU_UNLOAD")
                return False
        finally:
            if loading_failed == False:
                self.fail_counter = 0
                gcmd.respond_info(f"MMU: Filament in Slot {slot} loaded")
                gcode.run_script_from_command(f"SAVE_VARIABLE VARIABLE=mmu_changing_slot VALUE=5")
                gcode.run_script_from_command(f"SAVE_VARIABLE VARIABLE=mmu_loaded_slot VALUE={slot}")
            else:
                gcode.run_script_from_command(f"SAVE_VARIABLE VARIABLE=mmu_loaded_slot VALUE=5")
            gcode.run_script_from_command("RESTORE_GCODE_STATE NAME=MMU_LOAD")
            if loading_failed == False:
                return True
            else:
                return False

    def cmd_MMU_UNLOAD(self, gcmd):
        tip_shaping = gcmd.get_int('TIP_SHAPE', 0)

        gcode = self.printer.lookup_object("gcode")
        sensor = self.printer.lookup_object("filament_switch_sensor fsensor")
        reactor = self.printer.get_reactor()

        # Check current status first
        while True:
            status = self.mmu.get_status()
            if status['status'] == 'Finished' or status['value'] == 0:
                break
            if status['status'] == 'Error':
                error_name = MMU_error_table.get(status['value'], f"UNKNOWN_ERROR_0x{status['value']:04X}")
                gcmd.respond_info(f"MMU: Unloading Error -> {error_name}")
                self._pause_print()
                return
            reactor.pause(reactor.monotonic() + 0.002)

        if self.mmu.read_register_retry(MMU_register_table["FINDA_State"]) == 0:
            gcmd.respond_info("MMU: FINDA probe not activated. Assuming filament not loaded.")
            return

        gcode.run_script_from_command(f"SAVE_VARIABLE VARIABLE=mmu_spooljoin_enable VALUE=0")
        gcmd.respond_info(f"MMU: Starting Filament Unload")

        feedrate = self.unload_retract_speed * 60.0
        moved = 0.0

        gcode.run_script_from_command("SAVE_GCODE_STATE NAME=MMU_UNLOAD")
        gcode.run_script_from_command("M83")  # Relative extrusion

        try:
            # Run tip shaping sequence first.
            if tip_shaping == 1:
                gcmd.respond_info("MMU: Performing tip shaping...")
                gcode.run_script_from_command("MMU_TIP_SHAPE")

            # Continue retracting until the extruder sensor clears.
            if "MMU_UNLOAD_SEQUENCE" not in gcode.gcode_handlers:
                while moved < self.unload_max_retract:
                    status = sensor.get_status(reactor.monotonic())

                    if not status.get("filament_detected", False):
                        gcmd.respond_info("MMU: Filament cleared extruder sensor")

                        # Engage MMU pulley.
                        self.mmu.request('f', 0)

                        # Hand off to the MMU.
                        self.mmu.request('U', 0)
                        while True:
                            status = self.mmu.get_status()
                            if status['status'] == 'Finished' or status['value'] == 0:
                                gcmd.respond_info(f"MMU: Unloaded Filament")
                                break
                            if status['status'] == 'Error':
                                error_name = MMU_error_table.get(status['value'], f"UNKNOWN_ERROR_0x{status['value']:04X}")
                                gcmd.respond_info(f"MMU: Unloading Error -> {error_name}")
                                self._pause_print()
                                return
                            reactor.pause(reactor.monotonic() + 0.002)
                        return

                    gcode.run_script_from_command(f"G1 E-{self.unload_step:.3f} F{feedrate:.0f}")
                    moved += self.unload_step

                gcmd.respond_info("MMU: Failed to clear extruder sensor after additional retract")
            else:
                self.mmu.request('f', 1)
                reactor.pause(reactor.monotonic() + 0.002)
                gcode.run_script_from_command(f"MMU_UNLOAD_SEQUENCE")
                status = sensor.get_status(reactor.monotonic())
                if status.get("filament_detected", False):
                    gcmd.respond_info("MMU: Failed to clear extruder sensor after additional retract")
                    self._pause_print()
                    return
                # Engage MMU pulley.
                self.mmu.request('f', 0)
                # Hand off to the MMU.
                self.mmu.request('U', 0)
                while True:
                    status = self.mmu.get_status()
                    if status['status'] == 'Finished' or status['value'] == 0:
                        gcmd.respond_info(f"MMU: Unloaded Filament")
                        break
                    if status['status'] == 'Error':
                        error_name = MMU_error_table.get(status['value'], f"UNKNOWN_ERROR_0x{status['value']:04X}")
                        gcmd.respond_info(f"MMU: Unloading Error -> {error_name}")
                        self._pause_print()
                        return
                    reactor.pause(reactor.monotonic() + 0.002)

        finally:
            gcode.run_script_from_command(f"SAVE_VARIABLE VARIABLE=mmu_loaded_slot VALUE=5")
            gcode.run_script_from_command("RESTORE_GCODE_STATE NAME=MMU_UNLOAD")

    def cmd_MMU_PRELOAD(self, gcmd):
        slot = gcmd.get_int('SLOT')
        self.mmu.request('L', slot)
        reactor = self.printer.get_reactor()
        while True:
            status = self.mmu.get_status()
            if status['status'] == 'Finished' or status['value'] == 0:
                break
            if status['status'] == 'Error':
                error_name = MMU_error_table.get(status['value'], f"UNKNOWN_ERROR_0x{status['value']:04X}")
                gcmd.respond_info(f"MMU: Preload Error -> {error_name}")
                return
            reactor.pause(reactor.monotonic() + 0.002)
        gcmd.respond_info(f"MMU: Preloaded Slot {slot}")

    def cmd_MMU_STATUS(self, gcmd):
        status = self.mmu.get_status()
        gcmd.respond_info(f"MMU: Status -> {status}")

    def cmd_MMU_REG_READ(self, gcmd):
        reg_entry = gcmd.get('REG_ENTRY', "")
        if reg_entry not in MMU_register_table:
            gcmd.respond_info(f"MMU: MMU_REG_READ required REG_ENTRY=reg syntax")
            valid_regs = []
            for key, _ in MMU_register_table.items():
                valid_regs.append(key)
                valid_regs_str = "\n".join(valid_regs)
            gcmd.respond_info(f"MMU: Valid Registers:\n{valid_regs_str}")
            return
        data = self.mmu.read_register_retry(addr=MMU_register_table[reg_entry])
        gcmd.respond_info(f"MMU: Read -> {data} for Register -> {reg_entry}")

    def cmd_MMU_CUT(self, gcmd):
        slot = gcmd.get_int('SLOT')
        sensor = self.printer.lookup_object("filament_switch_sensor fsensor")
        reactor = self.printer.get_reactor()
        if sensor.get_status(reactor.monotonic()).get("filament_detected", False):
            gcmd.respond_info("MMU: Not cutting. Extruder filament sensor is active")
            return
        self.mmu.request('K', slot)
        while True:
            status = self.mmu.get_status()
            if status['status'] == 'Finished' or status['value'] == 0:
                break
            if status['status'] == 'Error':
                error_name = MMU_error_table.get(status['value'], f"UNKNOWN_ERROR_0x{status['value']:04X}")
                gcmd.respond_info(f"MMU: Cut Error -> {error_name}")
                return
            reactor.pause(reactor.monotonic() + 0.002)
        gcmd.respond_info(f"MMU: Filament Cut for Slot {slot}")

    def cmd_MMU_RESET(self, gcmd):
        gcode = self.printer.lookup_object("gcode")
        reactor = self.printer.get_reactor()
        self.mmu.request('X', 0)
        gcode.run_script_from_command(f"SAVE_VARIABLE VARIABLE=mmu_changing_slot VALUE=5")
        gcode.run_script_from_command(f"SAVE_VARIABLE VARIABLE=mmu_spooljoin_enable VALUE=0")
        while True:
            status = self.mmu.get_status()
            if status['status'] == 'Finished' or status['value'] == 0:
                break
            if status['status'] == 'Error':
                error_name = MMU_error_table.get(status['value'], f"UNKNOWN_ERROR_0x{status['value']:04X}")
                gcmd.respond_info(f"MMU: Reset Error -> {error_name}")
                return
            reactor.pause(reactor.monotonic() + 0.002)
        fil_state = self.mmu.read_register_retry(addr=MMU_register_table["Filament_State"])
        if fil_state == 1:
            gcode.run_script_from_command(f"SAVE_VARIABLE VARIABLE=mmu_loaded_slot VALUE=5")

        if self.mmu_slow_feedrate != 0: # Write register to update slow feedrate of MMU
            self.mmu.write_register(MMU_register_table['pulley_slow_feedrate'], self.mmu_slow_feedrate)

        if self.mmu_load_feedrate != 0: # Write register to update load feedrate of MMU
            self.mmu.write_register(MMU_register_table['pulley_load_feedrate'], self.mmu_load_feedrate)

        if self.mmu_bowden_length != 0: # Write register to update bowden length of MMU
            self.mmu.write_register(MMU_register_table['bowden_length'], self.mmu_bowden_length)

        if self.mmu_cut_length != 0: # Write register to update cut length of MMU
            self.mmu.write_register(MMU_register_table['cut_length'], self.mmu_cut_length)

    def save_spooljoin(self, spool_list):
        sv = self.printer.lookup_object("save_variables")
        gcode = self.printer.lookup_object("gcode")

        literal = repr(spool_list)
        sv_var = gcode.create_gcode_command("SAVE_VARIABLE", "", {"VARIABLE": "mmu_spooljoin", "VALUE": literal})
        sv.cmd_SAVE_VARIABLE(sv_var)

    def cmd_MMU_SET_SPOOLJOIN(self, gcmd):
        raw_order = gcmd.get("ORDER", "")
        if raw_order is None:
            gcmd.respond_info(f"MMU: MMU_SET_SPOOLJOIN required ORDER=1,2,3,4,5 syntax most significant first")
            return

        try:
            spool_list = [int(x.strip()) for x in raw_order.split(",") if x.strip()]
        except Exception:
            gcmd.respond_info(f"MMU: MMU_SET_SPOOLJOIN required ORDER=1,2,3,4,5 syntax most significant first")
            return

        self.save_spooljoin(spool_list)
# ------------------------------------------------------------
# Klipper entry point
# ------------------------------------------------------------
def load_config(config):
    return MMU2S_Klipper(config)
