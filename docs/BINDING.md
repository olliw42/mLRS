# mLRS Documentation: Binding #

([back to main page](../README.md))

For a Tx module and a receiver to be able to connect to each other, some parameters (the common parameters) need to be set to equal values in both the Tx module and receiver. For an arbitrary receiver this can be achieved by binding.

The binding procedure is as follows:

1. On the Tx side you have three options: (i) press the bind button on the Tx module, (ii) initiate the binding via a CLI command, or (iii) initiate the binding via the mLRS configuration Lua script.
2. Press the bind button on the receiver.

The sequence doesn't matter, i.e., one also can first press the bind button on the receiver and the set the Tx module into binding mode.

When a receiver is connected to a Tx module, the common parameters (as well as all other parameters) can be changed and be made active by issuing a Save either via the CLI or the mLRS configuration Lua script.