# mLRS Documentation: CLI Commands #

([back to main page](../README.md))

Besides the mLRS Lua configuration script, which however is not available for all radio transmitters, the CLI is the main method for configuring the mLRS transmitter and receiver modules.

The CLI commands consist of one or more strings, each separated by a blank, and a terminating character. The terminating character can be '\r' (carriage return), '\n' (line feed), ',' or ';'. The CR/LF line feed handling depends very much on the terminal which is being used and its configuration. Hence, it is good practice to just always terminate the CLI commands with, e.g., a ';' (this will be done for the commands listed below). The CLI is case sensitive, except for parameter names. That is, the commands need to be entered lower case, but parameter names can be enterd with any caseing.

The serial settings are: baudrate 115200 bps, 1 stop bit, no parity.

Depending on the device, some parameters are not available for configuration or cannot be changed. For instance, for a device which doesn't support a buzzer the parameter "Buzzer" is not available, and for a device which doesn't support diversity the parameter "Diversity" cannot be changed. Parameters which are not available are displayed with a value "-", and parameters which cannot be changed are displayed with the current selection but a comment "(unchangeable)" in addition.


## Commands ##

you should be able to copy and paste these commands, to change a setting just add = 0123etc

p Tx_Cli_LineEnd =3;

Commands

h; or help; or ?; Lists the available commands, with a very brief description

pl; Lists all parameters and their settings.

pl c; Lists the parameters shared between tx and rx,

pl tx; Lists the parameters of the Tx module and their settings.
pl rx; Lists the parameters of the Rx module and their settings.

pstore; Stores the new parameter settings into the Tx module and, if connected, also into the receiver.

bind; Starts the binding. Comment: Only the Tx module is set into binding mode. The receiver must be put into binding mode by pressing its bind button.

reload; Reloads the current parameter values from the Tx and, if connected, the receiver.

stats; Starts streaming some statistics. Terminate by sending any character.

Common Parameters
p Bind_Phrase; String of 6 characters. The characters can be 'a'-'z', '0'-'9', '_', '#', '-', '.',

p Mode; Operation mode. Can be "50 Hz", "31 Hz", "19 Hz".

p RF_Band; Frequency band. May not be selectable.


Tx Parameters

p Tx_Power; Transmission power in Watts.

p Tx_Diversity; Diversity mode. Can be "enabled", "antenna1", "antenna2".

p Tx_Ch_Source; Selects the source from which the rc data should be read. Can be "none", "mbridge", "in", "crsf".

p Tx_Ch_Order; Channel order of the rc data provided to the Tx module. Can be "AETR", "TAER", "ETAR".
p Tx_In_Mode; Selects the protocol of the rc data on the in port. Effective only when "Tx Ch Source" = "in". Can be "sbus", "sbus inv".

p Tx_Ser_Dest; Selects the destination/source of the serial data stream. Can be "serial", "mbridge", "serial2"

p Tx_Ser_Baudrate; Baudrate of the serial data stream. Effective only for "Tx Ser Dest" = "serial" or "serial2". Can be "9600", "19200", "38400", "57600", "115200".

p Tx_Ser_Link_Mode; Selects how the serial data stream is processed. Can be "transp.", "mavlink".

p Tx_Snd_RadioStat; Determines if a MAVLink RADIO_STATUS message is emitted by the Tx module, and what txbuf mechanism is used. Effective only when "Tx Ser Link Mode" = "mavlink". Can be "off", "on", "on w txbuf".

p Tx_Buzzer; Enables the buzzer, and selects what data it reflects. Can be "off", "LP", "rxLQ".

p Tx_Cli_LineEnd; Determines the line termination character(s) used by the CLI. Can be "CR", "LF", "CRLF".

Rx Parameters


p Rx_Power; Transmission power in Watts.

p Rx_Diversity; Diversity mode. Can be "enabled", "antenna1", "antenna2".

p Rx_Ch_Order; Channel order of the rc data emitted by the receiver. Can be "AETR", "TAER", "ETAR".

p Rx_Out_Mode; Selects the protocol of the rc data emitted on the out port. Can be "sbus", "crsf", "sbus inv".

p Rx_Out_Rssi_Ch; Determines if and on which channel the RSSI value is send out. Can be "off", "5" - "12".

p Rx_FailSafe_Mode; Determines the behavior upon a failsafe. Can be "no sig", "low thr", "by cnf", "low thr cnt", "ch1ch4 cnt".

p Rx_Ser_Baudrate; Baudrate of the serial data stream. Can be "9600", "19200", "38400", "57600", "115200", "230400".

p Rx_Ser_Link_Mode; Selects how the serial data stream is processed. Can be "transp.", "mavlink".

p Rx_Snd_RadioStat; Determines if a MAVLink RADIO_STATUS or RADIO_LINK_FLOW_CONTROL message is emitted by the receiver, and what txbuf mechanism is used. Effective only when "Rx Ser Link Mode" = "mavlink". Can be "off", "on", "on w txbuf". A RADIO_LINK_FLOW_CONTROLis emitted if "Rx Snd RcChannel" is set to "rc channels".

p Rx_Snd_RcChannel; Determines if a MAVLink RC_CHANNELS_OVERRIDE or RADIO_RC_CHANNELS message is emitted by the receiver. Effective only when "Rx Ser Link Mode" = "mavlink". Can be "off", "rc override", "rc channels".

p Rx_Buzzer; Enables the buzzer, and selects what data it reflects. Can be "off", "LP".

p Rx_FS_Ch1 - Rx_FS_Ch16; Sets the rc channel value upon a failsafe. Effective only when "Rx FailSafe Mode" = "by cnf". Can be a value between -120% and +120%.

