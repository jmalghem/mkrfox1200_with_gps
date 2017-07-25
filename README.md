# Envoi des coordonnées GPS par SigFox avec un MRKFOX1200

SigFox backend Custom Payload :
Lat::float:32:little-endian Lng::float:32:little-endian device_id::uint:16:little-endian voltage::uint:16:little-endian

Sample valid frame :
$GPRMC,142853.00,A,4306.31491,N,00137.62595,E,1.395,,250717,,,A*76

GPS Receiver used : SparkFun GPS Receiver - GP-20U7 (56 Channel) - https://www.sparkfun.com/products/13740
