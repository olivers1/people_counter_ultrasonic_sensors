import RPi.GPIO as GPIO
from hcsr04sensor import sensor
from enum import Enum

# set gpio pins
trig_sensor1 = 22
echo_sensor1 = 23
trig_sensor2 = 17
echo_sensor2 = 27

class Sensor1Status(Enum):
    NO_TRIGG = 0
    TRIGG = 1

class Sensor2Status(Enum):
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

class ScenarioDetected(Enum):
    NO_SCENARIO = 0
    EXIT = 1
    ENTER = 2


class UltrasonicSensor:
    def __init__(self, trig_pin, echo_pin, n_samples=11, sample_wait=0.1):
        self._trig_pin = trig_pin
        self._echo_pin = echo_pin
        self._n_samples = n_samples
        self._sample_wait = sample_wait

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)  # use GPIO.BOARD for board pin values

        self._sensor1 = sensor.Measurement(self._trig_pin, self._echo_pin)
        print("constructor")
 
    def __del__(self):
        # cleanup gpio pins.
        GPIO.cleanup((self._trig_pin, self._echo_pin))
        print("deconstructor")
    
    def read_distance(self):
        return round(self._sensor1.raw_distance(self._n_samples, self._sample_wait), 1)



def run():
    # create sensors
        sensor1 = UltrasonicSensor(trig_sensor1, echo_sensor1, 6, 0.07)
        sensor2 = UltrasonicSensor(trig_sensor2, echo_sensor2, 6, 0.07) 
    

def main(args):
    run()

    return 0

if __name__ == '__main__':
    import sys
    sys.exit(main(sys.argv))
