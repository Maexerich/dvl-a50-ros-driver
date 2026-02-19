Header header
#Time since last velocity report in milliseconds
float64 time_since_last_data_ms
#Measured velocity [m/s]
geometry_msgs/Vector3 velocity
#Figure of Merit
float64 fom
#Altitude of the sensor
float64 altitude
#Beams/Transducers
DVLBeam[] beams
#Validity of velocity
bool velocity_valid
#Status message
int64 status
#Formatting of json
string form
#Covariance of doppler velocity
float64[9] covariance
#Time of validity: Timestamp of surface reflection (microseconds)
uint64 time_of_validity_us
#Time of transmission (microseconds)
uint64 time_of_transmission_us
