Advanced Weather Station V2
The Advanced Weather Station V2 is a high-precision environmental monitoring system designed for professional-grade reliability and scientific accuracy. Building upon the success of the first version, V2 introduces significant hardware optimizations and a more powerful processing architecture to deliver even better performance in the field.
![TOP](https://github.com/user-attachments/assets/00fcd6bc-de0a-4cc9-88d0-b507ecf0fa58)
![BOTTOM](https://github.com/user-attachments/assets/7c3ae1fb-cf95-427d-9bab-6407a5c004b4)

Key Improvements in V2
Upgraded MCU: Now powered by the STM32G431, providing enhanced processing capabilities, better power efficiency, and advanced peripherals compared to the previous generation.

Streamlined Communication: All Programmable Gain Amplifiers (PGAs) are now consolidated on a single UART bus, simplifying the communication protocol and reducing cable complexity.

Enhanced PCB Design: A completely overhauled PCB layout featuring improved signal integrity, better thermal management, and a more compact, robust form factor for long-term outdoor deployment.

Overview
This weather station integrates a comprehensive suite of high-precision sensors to monitor atmospheric conditions, air quality, and spatial positioning. It leverages ultrasonic technology for maintenance-free wind measurement and GNSS RTK for centimetre-level location accuracy.

Features & Sensors
Ultrasonic Wind Measurement: Precise wind speed and direction without moving parts.

GNSS RTK: High-precision positioning for mobile or stationary research.

Environmental Suite:

UV & Ambient Light: UV radiation and light intensity monitoring.

Air Quality: ScioSense ENS160 for CO2 and VOC detection.

Lightning Detection: ScioSense AS3935 for storm activity tracking.

Pressure & Temp: Bosch BMP581 for barometric accuracy.

Humidity: Dual TI HDC3020 sensors for redundant climate analysis.

Rugged Construction: Optical sensors are protected by UV-transparent glass, and the ultrasonic assembly uses 304 stainless steel and ASA 3D-printed enclosures.

Workflow
Data Acquisition: Sensors collect raw environmental data, including wind vectors, gas concentrations, and radiation levels.

Processing: The STM32G431 aggregates the data, applies calibration constants, and handles temperature compensation in real-time.

Connectivity: Processed data is transmitted via an ESP32-PoE interface to Ethernet networks, enabling seamless integration with MQTT brokers or cloud-based dashboards.

Applications
Climate & Scientific Research: Accurate data collection for long-term studies.

Smart Agriculture: Optimize irrigation and crop protection based on hyper-local weather.

Renewable Energy: Ideal for solar tracking and wind farm site assessment.

Smart Cities: Real-time air quality and environmental safety monitoring.
