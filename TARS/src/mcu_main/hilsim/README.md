HILSIM Server
-----
HILSIM server is a way to automatically test HILSIM.

Open up this folder in a terminal, run `python flask_app.py`, and then you can start sending post requests with `post_request.py`.

The output of HILSIM will be dumped in post_return.txt after a couple of minutes of running.

## Errors
If there are many "Got nothing"s at the start of post_return.txt, it is fine, this is just HILSIM waiting for packets.

If you see many "Wrong wire type" error messages, the protobuf packet might be outdated, so follow the instructions in hilsimpacket.proto to rewrite the packets.

If that does not fix it, it might be an issue with the size of the packet or alignment of the packets. Double check the size of the packet, it should be 70 bytes. It might have changed as we readjust the data we send to HILSIM. So create a protobuf object and print out `len(hilsimpacket.SerializeToString())`.

Then, take that value, and change `packet_size` on line 62 (subject to change) in `main.cpp`. 
