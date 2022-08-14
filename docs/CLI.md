# mLRS Documentation. CLI Commands#

([back to main page](../README.md))

When the transmitter is not flashed with the MAVLink for OpenTx firmware, the CLI is currently the main and sole method for configuring the transmitter and receiver module.

The CLI commands consist of one or more strings, each separated by a blank, and a terminating character. The terminating character can be a '\r' (carriage return), '\n' (line feed), ',' or ';'. The line feed handling depends very much on the terminal which is being used and/or its configuration. Hence, it is good practice to just always terminate the CLI command with, e.g., a ';' (this will be done for the commands listed here). 

## Commands ##

#### h; or help; or ?; #### 
Lists the available commands, with a very brief description

#### pl; #### 
Lists all parameters and their settings. Comment: The parameters of the receiver are listed only if a receiver is connected, else a warning messages is printed.

#### pl c; #### 
Lists the shared parameters, i.e., those parameters which are common for both the Tx module and the receiver, and their settings. Comment: If a receiver is not connected, a warning messages is printed.

#### pl tx; #### 
Lists the parameters of the Tx module and their settings. 

#### pl rx; #### 
Lists the parameters of the receiver and their settings. Comment: These parameters are listed only if a receiver is connected, else a warning message is printed.

#### p name; or p name = ?; #### 
Prints the setting of the parameter with name 'name'. Comment: Blanks in the parameter name should be replaced by '_'. E.g., the setting for the parameter "Tx Power" would be obtained with the CLI command p tx_power;.

#### p name = value; #### 
Changes the setting of the parameter with name 'name' to the specified value. Comment: Blanks in the parameter name should be replaced by '_'. E.g., the setting for the parameter "Tx Power" would be changed to zero with the CLI command p tx_power = 0;.

#### pstore; #### 
Stores the new parameter settings into the Tx module and, if connected, also into the receiver.

#### bind; #### 
Starts the binding. Comment: Only the Tx module is set into binding mode. The receiver must be put into binding mode by pressing its bind button.

#### reload; #### 
Reloads the current parameter values from the Tx and, if connected, the receiver.

#### stats; #### 
Starts streaming some statistics. Terminate by sending any character.

