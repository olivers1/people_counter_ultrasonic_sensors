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
        GPIO.cleanup(self._trig_pin, self._echo_pin)
        print("deconstructor")
    
    def read_distance(self):
        return round(self._sensor1.raw_distance(self._n_samples, self._sample_wait), 1)


class SensorHandler(UltrasonicSensor):
    def __init__(self, trigg_dist_treshold=10):
        self._trigg_dist_treshold = trigg_dist_treshold
        # create sensors
        self._sensor1 = UltrasonicSensor(trig_sensor1, echo_sensor1, 6, 0.07)
        self._sensor2 = UltrasonicSensor(trig_sensor2, echo_sensor2, 6, 0.07) 

        self._initial_dist_sensor1 = 0
        self._initial_dist_sensor2 = 0

    def calibrate_sensor(self):
        sensor1_values = []
        sensor2_values = []
        for _ in range(5):
            sensor1_values.append(self._sensor1.read_distance())
            sensor2_values.append(self._sensor2.read_distance())
        print("sensor1:", *sensor1_values)
        print("sensor2:", *sensor2_values)

        # evaluate calibration
        sensor1_mean = sum(sensor1_values) / len(sensor1_values)
        sensor2_mean = sum(sensor2_values) / len(sensor2_values)

        # add reference distance values for sensors
        self._initial_dist_sensor1 = self._sensor1.read_distance()
        self._initial_dist_sensor2 = self._sensor2.read_distance()
        
        # verify measured data is 
        if self._initial_dist_sensor1 - sensor1_mean > 5 or self._initial_dist_sensor2 - sensor2_mean > 5:
            raise SystemError("Sensor calibration failed")

    def detect_motion_pattern(self):
        # set/reset enums
        sensor1_status = Sensor1Status.NO_TRIGG
        sensor2_status = Sensor2Status.NO_TRIGG
        trigg_first_sensor = TriggFirstSensor.NO_TRIGG
        trigg_second_sensor = TriggSecondSensor.NO_TRIGG
        untrigg_first_sensor = UntriggFirstSensor.NO_TRIGG
        untrigg_second_sensor = UntriggSecondSensor.NO_TRIGG
        captured_motion_pattern = CapturedMotionPattern.NO_PATTERN
        # read initial sensor values
        sensor1_value = self._sensor1.read_distance()
        sensor2_value = self._sensor2.read_distance()

        try:
            while trigg_first_sensor == TriggFirstSensor.NO_TRIGG or trigg_second_sensor == TriggSecondSensor.NO_TRIGG:
                sensor1_value = self._sensor1.read_distance()
                sensor2_value = self._sensor2.read_distance()
                
                if abs(sensor1_value - self._initial_dist_sensor1) > self._trigg_dist_treshold:   
                    sensor1_status = Sensor1Status.TRIGG
                    sensor2_status = Sensor2Status.NO_TRIGG
                    trigg_first_sensor = TriggFirstSensor.SENSOR_1
                    if abs(sensor2_value - self._initial_dist_sensor2) > self._trigg_dist_treshold:
                        sensor2_status = Sensor2Status.TRIGG
                        trigg_second_sensor = TriggSecondSensor.SENSOR_2
                elif abs(sensor2_value - self._initial_dist_sensor2) > self._trigg_dist_treshold: 
                    sensor2_status = Sensor2Status.TRIGG 
                    sensor1_status = Sensor1Status.NO_TRIGG
                    trigg_first_sensor = TriggFirstSensor.SENSOR_2  
                    if abs(sensor1_value - self._initial_dist_sensor1) > self._trigg_dist_treshold:
                        sensor1_status = Sensor1Status.TRIGG
                        trigg_second_sensor = TriggSecondSensor.SENSOR_1
            
            while trigg_first_sensor != TriggFirstSensor.NO_TRIGG or trigg_second_sensor != TriggSecondSensor.NO_TRIGG:
                sensor1_value = self._sensor1.read_distance()
                sensor2_value = self._sensor2.read_distance() 

                if abs(sensor1_value - self._initial_dist_sensor1) < self._trigg_dist_treshold:   
                    sensor1_status = Sensor1Status.NO_TRIGG
                    untrigg_first_sensor = UntriggFirstSensor.NO_TRIGG
                    if abs(sensor2_value - self._initial_dist_sensor2) < self._trigg_dist_treshold:
                        sensor2_status = Sensor2Status.NO_TRIGG
                        untrigg_second_sensor = UntriggSecondSensor.NO_TRIGG
                        captured_motion_pattern = CapturedMotionPattern.EXITING
                elif abs(sensor2_value - self._initial_dist_sensor2) < self._trigg_dist_treshold: 
                    sensor2_status = Sensor2Status.NO_TRIGG 
                    untrigg_first_sensor = UntriggFirstSensor.NO_TRIGG
                    if abs(sensor1_value - self._initial_dist_sensor1) > self._trigg_dist_treshold:
                        sensor1_status = Sensor1Status.NO_TRIGG
                        untrigg_second_sensor = UntriggSecondSensor.NO_TRIGG
                        captured_motion_pattern = CapturedMotionPattern.ENTERING
            
            print("captured motion pattern:", captured_motion_pattern.value)

        except ValueError: #KeyboardInterrupt:
            print('interrupted!')

def run():
    sensor_handler = SensorHandler()
    sensor_handler.calibrate_sensor()
    sensor_handler.detect_motion_pattern()

def main(args):
    run()

    return 0

if __name__ == '__main__':
    import sys
    sys.exit(main(sys.argv))
