import RPi.GPIO as GPIO
from hcsr04sensor import sensor
from enum import Enum
import numpy as np

# set gpio pins
SENSOR_1_TRIG = 22
SENSOR_1_ECHO = 23
SENSOR_2_TRIG = 17
SENSOR_2_ECHO = 27

# sample configuration
N_SAMPLES = 6
SAMPLE_WAIT = 0.07

# calibration
N_CALIBRATION_SAMPLES = 6
SENSOR_DISTANCE_VARIATION = 2

# trigg threshold
DISTANCE_TRESHOLD = 5

# status enums
class Sensor_1_State(Enum):
    NO_TRIGG = 0
    TRIGG = 1

class Sensor_2_State(Enum):
    NO_TRIGG = 0
    TRIGG = 1

class Sensor_Calibration_Sts(Enum):
    UNKNOWN = 0
    CALIBRATED = 1
    CALIBRATION_FAILED = 2

class FirstSensorTriggSts(Enum):
    UNKNOWN = 0
    SENSOR_1 = 1
    SENSOR_2 = 2        

class SecondSensorTriggSts(Enum):
    UNKNOWN = 0
    SENSOR_1 = 1
    SENSOR_2 = 2

class FirstSensorUntriggSts(Enum):
    UNKNOWN = 0
    SENSOR_1 = 1
    SENSOR_2 = 2
        
class SecondSensorUntriggSts(Enum):
    UNKNOWN = 0
    SENSOR_1 = 1
    SENSOR_2 = 2

class DetectedScenario(Enum):
    NO_SCENARIO = 0
    EXIT = 1
    ENTER = 2


class UltrasonicSensor:
    def __init__(self, trig_pin, echo_pin, n_samples=11, sample_wait=0.1):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)  # use GPIO.BOARD for board pin values
        self._sensor_1_value = 0
        self._sensor_2_value = 0
        # initiate status enums
        self._sensor_1_state = Sensor_1_State.NO_TRIGG
        self._sensor_2_state = Sensor_2_State.NO_TRIGG 
        self._sensor_calibration_sts = Sensor_Calibration_Sts.UNKNOWN
        self._first_sensor_trigg_sts = FirstSensorTriggSts.UNKNOWN
        self._second_sensor_trigg_sts = SecondSensorTriggSts.UNKNOWN
        self._first_sensor_untrigg_sts = FirstSensorUntriggSts.UNKNOWN
        self._second_sensor_untrigg_sts = SecondSensorUntriggSts.UNKNOWN 
        self._detected_scenario = DetectedScenario.NO_SCENARIO

        self._trig_pin = trig_pin
        self._echo_pin = echo_pin
        self._sensor_obj = sensor.Measurement(self._trig_pin, self._echo_pin)
        print("constructor")

    def calibrate(self):
        sensor_values = []

        for _ in range(N_CALIBRATION_SAMPLES):
            sensor_values.append(self.read_distance())

        # print calibration read-outs
        print("sensor:", *sensor_values)

        # evaluate calibration
        sensor_mean = sum(sensor_values) / len(sensor_values)
        arr = np.asarray(sensor_values)     # convert list to array with numpy package

        # assign closest readout value as reference distance value
        index = (np.abs(arr - sensor_mean)).argmin()
        ref_distance_sensor = sensor_values[index]
        
        # verify if measured data is ok by comparing mean value with selected reference value
        if ref_distance_sensor - sensor_mean > SENSOR_DISTANCE_VARIATION:
            self._sensor_calibration_sts = Sensor_Calibration_Sts.CALIBRATION_FAILED
            raise ValueError("no stable reference data")
        else:
            self._sensor_calibration_sts = Sensor_Calibration_Sts.CALIBRATED
        
        return ref_distance_sensor

    def read_distance(self):
        return round(self._sensor_obj.raw_distance(N_SAMPLES, SAMPLE_WAIT), 1)

    def detect_sensor_trigg(self, sensor_1, sensor_2):
        # read sensors
        self._sensor_1_value = sensor_1.read_distance()
        self._sensor_2_value = sensor_2.read_distance() 
        if abs(self._sensor_1_value - self._ref_distance_sensor_1) > DISTANCE_TRESHOLD:
            self._sensor_1_state = Sensor_1_State.TRIGG
        elif abs(self._sensor_2_value - self._ref_distance_sensor_2) > DISTANCE_TRESHOLD:
            self._sensor_2_state = Sensor_2_State.TRIGG
        else:
            self._sensor_1_state = Sensor_1_State.NO_TRIGG
            self._sensor_2_state = Sensor_2_State.NO_TRIGG
    
    def detect_motion(self, sensor_1, sensor_2):
        # detect first and second sensor triggs
        if self._sensor_calibration_sts == Sensor_Calibration_Sts.CALIBRATED and self._first_sensor_trigg_sts == FirstSensorTriggSts.UNKNOWN:
            while self._first_sensor_trigg_sts == FirstSensorTriggSts.UNKNOWN and self._second_sensor_trigg_sts == SecondSensorTriggSts.UNKNOWN:
                self.detect_sensor_trigg(sensor_1, sensor_2)
                if self._sensor_1_state == Sensor_1_State.TRIGG:
                    self._first_sensor_trigg_sts = FirstSensorTriggSts.SENSOR_1
                    if self._sensor_2_state == Sensor_2_State.TRIGG:
                        self._second_sensor_trigg_sts = SecondSensorTriggSts.SENSOR_2
                elif self._sensor_2_state == Sensor_2_State.TRIGG:
                    self._first_sensor_trigg_sts = FirstSensorTriggSts.SENSOR_2
                    if self._sensor_1_state == Sensor_1_State.TRIGG:
                        self._second_sensor_trigg_sts = SecondSensorTriggSts.SENSOR_1
                print("first sensor trigg:", self._first_sensor_trigg_sts.name)
                print("second sensor trigg:", self._second_sensor_trigg_sts.name)
        #elif self._first_sensor_trigg_sts != FirstSensorTriggSts.UNKNOWN and self._second_sensor_trigg_sts != FirstSensorTriggSts.UNKNOWN

        



class SensorHandler(UltrasonicSensor):
    def __init__(self):
        # setup and reset enums
        

        # create sensor objects
        self._sensor_1 = UltrasonicSensor(SENSOR_1_TRIG, SENSOR_1_ECHO, N_SAMPLES, SAMPLE_WAIT)
        self._sensor_2 = UltrasonicSensor(SENSOR_2_TRIG, SENSOR_2_ECHO, N_SAMPLES, SAMPLE_WAIT)

        # calibrate sensors
        self._ref_distance_sensor_1 = self._sensor_1.calibrate()
        self._ref_distance_sensor_2 = self._sensor_2.calibrate() 
        print("ref_distance_sensor_1:", self._ref_distance_sensor_1)
        print("ref_distance_sensor_2:", self._ref_distance_sensor_2)

        # detect motion
        self._sensor_1.detect_motion(self._sensor_1, self._sensor_2)
 




def run():
    sensor_handler = SensorHandler()
 
        
    

def main(args):
    run()

    return 0

if __name__ == '__main__':
    import sys
    sys.exit(main(sys.argv))
