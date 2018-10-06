set(COMPONENT_SRCS "voltimetro/Voltimetro.cpp"
				   "Parser/parser.cpp"
				   "Motors/Motors.cpp"
				   "Bluetooth/bluetooth.cpp"
				   "PIDControl/PIDController.cpp"
				   "main.cpp")
set(COMPONENT_ADD_INCLUDEDIRS ". include")

register_component()
