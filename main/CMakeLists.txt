set(COMPONENT_SRCS 	"Utils/Utils.cpp"
					"voltimetro/Voltimetro.cpp"
				    "Gyro/gyro.cpp"
				    "Parser/parser.cpp"
				    "Motors/Motors.cpp"
				    "Bluetooth/bluetooth.cpp"
				    "PIDControl/PIDController.cpp"
				    "Memory/Memory.cpp"
				    "main.cpp")
set(COMPONENT_ADD_INCLUDEDIRS ". include")

register_component()
