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

# status enums
class Sensor_1_State(Enum):
    NO_TRIGG = 0
    TRIGG = 1

class Sensor_2_State(Enum):
    NO_TRIGG = 0
    TRIGG = 1

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
        self._ref_distance_sensor = 0
        self._trig_pin = trig_pin
        self._echo_pin = echo_pin
        self._sensor_obj = sensor.Measurement(self._trig_pin, self._echo_pin)
        print("constructor")

    def calibrate(self, n_cal_samples, mean_difference):
        sensor_values = []

        for _ in range(n_cal_samples):
            sensor_values.append(self.read_distance())

        # print calibration read-outs
        print("sensor:", *sensor_values)

        # evaluate calibration
        sensor_mean = sum(sensor_values) / len(sensor_values)
        arr = np.asarray(sensor_values)     # convert list to array for numpy package

        # add reference distance value that is closest to mean value from the calibration of the sensor
        index = (np.abs(arr - sensor_mean)).argmin()
        self._ref_distance_sensor = sensor_values[index]
        
        # verify if measured data is ok by comparing mean value with selected reference value
        if self._ref_distance_sensor - sensor_mean > mean_difference:
            raise ValueError("no stable reference data")




    def read_distance(self):
        return round(self._sensor_obj.raw_distance(N_SAMPLES, SAMPLE_WAIT), 1)


class SensorHandler(UltrasonicSensor):
    def __init__(self):
        # setup and reset enums
        sensor_1_state = Sensor_1_State.NO_TRIGG
        sensor_2_state = Sensor_2_State.NO_TRIGG 
        first_sensor_trigg_sts = FirstSensorTriggSts.UNKNOWN
        second_sensor_trigg_sts = SecondSensorTriggSts.UNKNOWN
        first_sensor_untrigg_sts = FirstSensorUntriggSts.UNKNOWN
        second_sensor_untrigg_sts = SecondSensorUntriggSts.UNKNOWN 
        detected_scenario = DetectedScenario.NO_SCENARIO

        # create sensor objects
        self._sensor_1 = UltrasonicSensor(SENSOR_1_TRIG, SENSOR_1_ECHO, N_SAMPLES, SAMPLE_WAIT)
        self._sensor_2 = UltrasonicSensor(SENSOR_2_TRIG, SENSOR_2_ECHO, N_SAMPLES, SAMPLE_WAIT)

        # calibrate sensors
        self._sensor_1.calibrate(10, 2)
        self._sensor_2.calibrate(10, 2) 




def run():
    sensor_handler = SensorHandler()
 
        
    

def main(args):
    run()

    return 0

if __name__ == '__main__':
    import sys
    sys.exit(main(sys.argv))
