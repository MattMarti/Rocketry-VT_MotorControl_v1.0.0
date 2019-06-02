#ifndef PACKET_INTERPRET_H_INCLUDED
#define PACKET_INTERPRET_H_INCLUDED


/* the purpose of this library is to verify packet integrity, and sort the packet into it's parts */


/*sorts into packet parts and defines namespace variables */
bool sortPacket(unsigned int buf[]);

/*double check that sync bytes are correct */
bool sanityCheck();

/*prints sorted packet */
void printAll();



#endif // HOST_PACKET_INTERPRET_H_INCLUDED
