# Helper script for manual y height probing
#
# Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, bisect

class ManualProbe:
    def __init__(self, config):
        self.printer = config.get_printer()
        # Register commands
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode_move = self.printer.load_object(config, "gcode_move")
        self.gcode.register_command('MANUAL_PROBE', self.cmd_MANUAL_PROBE,
                                    desc=self.cmd_MANUAL_PROBE_help)
        yconfig = config.getsection('stepper_y')
        self.y_position_endstop = yconfig.getfloat('position_endstop_y', None,
                                                   note_valid=False)
        if self.y_position_endstop is not None:
            self.gcode.register_command(
                'Y_ENDSTOP_CALIBRATE', self.cmd_Y_ENDSTOP_CALIBRATE,
                desc=self.cmd_Y_ENDSTOP_CALIBRATE_help)
            self.gcode.register_command(
                'Y_OFFSET_APPLY_ENDSTOP',
                self.cmd_Y_OFFSET_APPLY_ENDSTOP,
                desc=self.cmd_Y_OFFSET_APPLY_ENDSTOP_help)
        self.reset_status()
    def manual_probe_finalize(self, kin_pos):
        if kin_pos is not None:
            self.gcode.respond_info("Y position is %.3f" % (kin_pos[1],))
    def reset_status(self):
        self.status = {
            'is_active': False,
            'y_position': None,
            'y_position_lower': None,
            'y_position_upper': None
        }
    def get_status(self, eventtime):
        return self.status
    cmd_MANUAL_PROBE_help = "Start manual probe helper script"
    def cmd_MANUAL_PROBE(self, gcmd):
        ManualProbeHelper(self.printer, gcmd, self.manual_probe_finalize)
    def y_endstop_finalize(self, kin_pos):
        if kin_pos is None:
            return
        y_pos = self.y_position_endstop - kin_pos[1]
        self.gcode.respond_info(
            "stepper_y: position_endstop_y: %.3f\n"
            "The SAVE_CONFIG command will update the printer config file\n"
            "with the above and restart the printer." % (y_pos,))
        configfile = self.printer.lookup_object('configfile')
        configfile.set('stepper_y', 'position_endstop_y', "%.3f" % (y_pos,))
    cmd_Y_ENDSTOP_CALIBRATE_help = "Calibrate a Y endstop"
    def cmd_Y_ENDSTOP_CALIBRATE(self, gcmd):
        ManualProbeHelper(self.printer, gcmd, self.y_endstop_finalize)
    def cmd_Y_OFFSET_APPLY_ENDSTOP(self,gcmd):
        offset_y = self.gcode_move.get_status()['homing_origin'].y
        configfile = self.printer.lookup_object('configfile')
        if offset_y == 0:
            self.gcode.respond_info("Nothing to do: Y offset_y is 0")
        else:
            new_calibrate_y = self.y_position_endstop - offset_y
            self.gcode.respond_info(
                "stepper_y: position_endstop_y: %.3f\n"
                "The SAVE_CONFIG command will update the printer config file\n"
                "with the above and restart the printer." % (new_calibrate_y))
            configfile.set('stepper_y', 'position_endstop_y',
                "%.3f" % (new_calibrate_y,))
    cmd_Y_OFFSET_APPLY_ENDSTOP_help = "Adjust the y endstop_position"

# Verify that a manual probe isn't already in progress
def verify_no_manual_probe(printer):
    gcode = printer.lookup_object('gcode')
    try:
        gcode.register_command('ACCEPT', 'dummy')
    except printer.config_error as e:
        raise gcode.error(
            "Already in a manual Y probe. Use ABORT to abort it.")
    gcode.register_command('ACCEPT', None)

Y_BOB_MINIMUM = 0.500
BISECT_MAX = 0.200

# Helper script to determine a Y height
class ManualProbeHelper:
    def __init__(self, printer, gcmd, finalize_callback):
        self.printer = printer
        self.finalize_callback = finalize_callback
        self.gcode = self.printer.lookup_object('gcode')
        self.toolhead = self.printer.lookup_object('toolhead')
        self.manual_probe = self.printer.lookup_object('manual_probe')
        self.speed = gcmd.get_float("SPEED", 5.)
        self.past_positions_y = []
        self.last_toolhead_pos = self.last_kinematics_pos = None
        # Register commands
        verify_no_manual_probe(printer)
        self.gcode.register_command('ACCEPT', self.cmd_ACCEPT,
                                    desc=self.cmd_ACCEPT_help)
        self.gcode.register_command('NEXT', self.cmd_ACCEPT)
        self.gcode.register_command('ABORT', self.cmd_ABORT,
                                    desc=self.cmd_ABORT_help)
        self.gcode.register_command('TESTY', self.cmd_TESTY,
                                    desc=self.cmd_TESTY_help)
        self.gcode.respond_info(
            "Starting manual Y probe. Use TESTY to adjust position.\n"
            "Finish with ACCEPT or ABORT command.")
        self.start_position = self.toolhead.get_position()
        self.report_y_status()
    def get_kinematics_pos(self):
        toolhead_pos = self.toolhead.get_position()
        if toolhead_pos == self.last_toolhead_pos:
            return self.last_kinematics_pos
        self.toolhead.flush_step_generation()
        kin = self.toolhead.get_kinematics()
        kin_spos = {s.get_name(): s.get_commanded_position()
                    for s in kin.get_steppers()}
        kin_pos = kin.calc_position(kin_spos)
        self.last_toolhead_pos = toolhead_pos
        self.last_kinematics_pos = kin_pos
        return kin_pos
    def move_y(self, y_pos):
        curpos = self.toolhead.get_position()
        try:
            y_bob_pos = y_pos + Y_BOB_MINIMUM
            if curpos[1] < y_bob_pos:
                self.toolhead.manual_move([None, y_bob_pos, None], self.speed)
            self.toolhead.manual_move([None, y_pos, None], self.speed)
        except self.printer.command_error as e:
            self.finalize(False)
            raise
    def report_y_status(self, warn_no_change=False, prev_pos_y=None):
        # Get position
        kin_pos = self.get_kinematics_pos()
        y_pos = kin_pos[1]
        if warn_no_change and y_pos == prev_pos_y:
            self.gcode.respond_info(
                "WARNING: No change in position (reached stepper resolution)")
        # Find recent positions that were tested
        pp = self.past_positions_y
        next_pos_y = bisect.bisect_left(pp, y_pos)
        prev_pos_y = next_pos_y - 1
        if next_pos_y < len(pp) and pp[next_pos_y] == y_pos:
            next_pos_y += 1
        prev_pos_y_val = next_pos_y_val = None
        prev_str_y = next_str_y = "??????"
        if prev_pos_y >= 0:
            prev_pos_y_val = pp[prev_pos_y]
            prev_str_y = "%.3f" % (prev_pos_y_val,)
        if next_pos_y < len(pp):
            next_pos_y_val = pp[next_pos_y]
            next_str_y = "%.3f" % (next_pos_y_val,)
        self.manual_probe.status = {
            'is_active': True,
            'y_position': y_pos,
            'y_position_lower': prev_pos_y_val,
            'y_position_upper': next_pos_y_val,
        }
        # Find recent positions
        self.gcode.respond_info("Y position: %s --> %.3f <-- %s"
                                % (prev_str_y, y_pos, next_str_y))
    cmd_ACCEPT_help = "Accept the current Y position"
    def cmd_ACCEPT(self, gcmd):
        pos = self.toolhead.get_position()
        start_pos = self.start_position
        if pos[:1] != start_pos[:1] or pos[1] >= start_pos[1]:
            gcmd.respond_info(
                "Manual probe failed! Use TESTY commands to position the\n"
                "nozzle prior to running ACCEPT.")
            self.finalize(False)
            return
        self.finalize(True)
    cmd_ABORT_help = "Abort manual Y probing tool"
    def cmd_ABORT(self, gcmd):
        self.finalize(False)
    cmd_TESTY_help = "Move to new Y height"
    def cmd_TESTY(self, gcmd):
        # Store current position for later reference
        kin_pos = self.get_kinematics_pos()
        y_pos = kin_pos[1]
        pp = self.past_positions_y
        insert_pos = bisect.bisect_left(pp, y_pos)
        if insert_pos >= len(pp) or pp[insert_pos] != y_pos:
            pp.insert(insert_pos, y_pos)
        # Determine next position to move to
        req = gcmd.get("Y")
        if req in ('+', '++'):
            check_y = 9999999999999.9
            if insert_pos < len(self.past_positions_y) - 1:
                check_y = self.past_positions_y[insert_pos + 1]
            if req == '+':
                check_y = (check_y + y_pos) / 2.
            next_y_pos = min(check_y, y_pos + BISECT_MAX)
        elif req in ('-', '--'):
            check_y = -9999999999999.9
            if insert_pos > 0:
                check_y = self.past_positions_y[insert_pos - 1]
            if req == '-':
                check_y = (check_y + y_pos) / 2.
            next_y_pos = max(check_y, y_pos - BISECT_MAX)
        else:
            next_y_pos = y_pos + gcmd.get_float("Y")
        # Move to given position and report it
        self.move_y(next_y_pos)
        self.report_y_status(next_y_pos != y_pos, y_pos)
    def finalize(self, success):
        self.manual_probe.reset_status()
        self.gcode.register_command('ACCEPT', None)
        self.gcode.register_command('NEXT', None)
        self.gcode.register_command('ABORT', None)
        self.gcode.register_command('TESTY', None)
        kin_pos = None
        if success:
            kin_pos = self.get_kinematics_pos()
        self.finalize_callback(kin_pos)

def load_config(config):
    return ManualProbe(config)
