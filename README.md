# NetworkCoding-ns-3.20

## Network Coding implementation on ns-3 ##

In this repository we present the following features, whose operation can be split into the following points:
	
	- **Network Coding** The cornerstone of the repo. We have added a bran new level between the transport and IP layers and implements the intra-flow network coding protocol to improve the performance exhibited by TCP over wireless mesh networks. This network coding protocol opts for UDP as the tranport level solution, providing a reliable transmission by means of the random linear coding of packets belonging to the same flow ("IntraflowNetworkCodingProtocol"), whose one of its main exponents is [1]). It is worth highlighting that we had to realy on different cross-layer mechanisms to perform the full protocol operation (namely, they are able to hace a direct interplay with some Wifi's sublayers, i.e "WififMacQueue")

	- **Scenario Creator**  In order to ease the construction of any scenario (since our project onlu focuses on wireless topologies, the unique physical layer with which are dealing with is based on IEEE 802.11b physical recommendation), we have developed a new module that allows the "offline" scenarios off-line configuration (without the need of a new compilation). Take a look at the documentation inside the module to have a better knowledge about how to generate scenarios through this tool

	- *Additional stuff* belonging to already existin sources (i.e MatrixErrorModel, ProprietaryTracing, etc.)

## External libraries ##

We have needed to use a number of functionalities that belongs to external libraries:
	
	1. IT++ ("lipitpp") C++ mathematical library. Namely, we will use its Galois Field GF(2) operations, since its performance is significatively higher that the one achieved by the previous one over the base field GF(2). The main drawback of this library is that it do not support all the matrix operations we need for extended GFs.
	2. M4RI/M4RIE ("libm4rie-dev"/"libm4ri-dev") Two methematical libraries that allows us to work with extended Galois Fields GF(2^q). 

	Both libraries can be installed through synaptic manager

## Instalation ##

In order no to overload the repository with too much unnecesary code, we have only included all the files that strictly belong to the ns-3.20 folder. Hence, it should be enough to compile and enjoy :-)

One of our main goals was not to tamper the legacy coded provided by the simulator. However, there hace been various situation in which we had to introduce "pieces of code" to allow correct operation of our solutions. Namely, we have tweaked some modules (i.e wifi, internet and network). In addition, it is worth highlighting that all the "new code" introduced is bounded within labels in order to have a quick location with the source files, as shown below. We mention this because on could try to merge our code with his/hers, hence it could lead to problems.

```
\\ Pablo\David\Ramon

NEW PROPRIETARY CODE

\\ End Pablo\David\Ramon

```

# Current Status ##

We have already provided many valoid results that have been included into various publications [2],[3],[4],[5]. 

## References ##
[1] [Chachulski, S., Jennings, M., Katti, S., & Katabi, D. (2007). Trading structure for randomness in wireless opportunistic routing (Vol. 37, No. 4, pp. 169-180). ACM.]
[2] [Gómez, D., Rodríguez, E., Agüero, R., & Muñoz, L. (2014, May). Reliable communications over wireless mesh networks with inter and intra-flow network coding. In Proceedings of the 2014 Workshop on ns-3 (p. 4). ACM.]
[3] [Gomez, D., Rodriguez, E., Aguero, R., & Munoz, L. (2014, June). Reliable communications over lossy wireless channels by means of the combination of UDP and Random Linear Coding. In Computers and Communication (ISCC), 2014 IEEE Symposium on (pp. 1-6). IEEE.]
[4] [Gomez, D., Garrido, P., Rodriguez, E., Aguero, R., & Munoz, L. (2014, November). Enhanced opportunistic random linear source/network coding with cross-layer techniques over wireless mesh networks. In Wireless Days (WD), 2014 IFIP (pp. 1-4). IEEE.]
[5] [Garrido Ortiz, P., Gomez, D., Aguero, R., & Munoz, L. Performance of Random Linear Coding over Multiple Error-Prone Wireless Links.]


## Contacts ##

For any question, criticism, doubt or curiosity, do not hesitate and contact us; we would be glad to provide an answer "ASAP"

* * *
Pablo Garrido Ortiz (<pgarrido@tlmat.unican.es>)
David Gómez Fernández (<dgomex@tlmat.unican.es>)
Ramón Agüero Calvo (<ramon@tlmat.unican.es>)
**[Universidad de Cantabria] (www.unican.es)**
2015
* * *
