set(COMPONENT_SRCS "Parser/parser.cpp"
				   "Bluetooth/bluetooth.cpp"
				   "PIDControl/PIDController.cpp"
				   "main.cpp")
set(COMPONENT_ADD_INCLUDEDIRS ". include")

register_component()
