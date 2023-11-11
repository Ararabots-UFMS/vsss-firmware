set(COMPONENT_SRCS 	"Utils/Utils.cpp"
					"voltimetro/Voltimetro.cpp"
				    "Parser/parser.cpp"
				    "Motors/Motors.cpp"
				    "Bluetooth/bluetooth_ble.cpp"
				    "PIDControl/PIDController.cpp"
				    "Memory/Memory.cpp"
					"EspNow/espnow.cpp"
				    "main.cpp")
set(COMPONENT_ADD_INCLUDEDIRS ". include")

register_component()
