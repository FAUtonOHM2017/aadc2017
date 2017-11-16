namespace cpp ext_iface

/******************************************************************************
 * interface objects
 ******************************************************************************/

/*! Defines what the binary in the DataRaw tranport is carrying
*/
enum TransportDef
{
	IMAGEDATA = 0, //raw data
}

/*! Generic transport struct.
*/
struct TROI
{
	1: required i16 x
	2: required i16 y
	3: required i16 width
	4: required i16 height
	5: required string name
}

struct TDataRaw
{
	1: required binary raw_data
	2: required list<TROI> rois
}

struct TDataClassification
{
	1: required string classification
	2: required double probability
}

struct TDataResult
{
	1: required TROI roi
	2: required list<TDataClassification> results
}

typedef list<TDataResult> TDataResultList

struct TImageParams
{
	1: required i16 height
	2: required i16 width
	3: required i16 bytesPerPixel
}

/** thrown by services */
exception TIoException {
    1: string message;
}

/**
 * Generic Service fuer communication between two thrift entities
 * @author wormerju
 */
service ExtService {

	/*!Sends a string to a partner, receives one in return.
	 * @return a nice welcome message with the service name - where you are.
	 * @throws TIoException
	 */
	string ping(1: string sender) throws (1: TIoException ioe);
	
	/*!Sends raw byte data, returns a bool upon return.
	*@return true on success, false if not
	*/
	TDataResultList rawData(1: TransportDef transport_def,  2: TDataRaw raw_data, 3: TImageParams params) throws (1: TIoException ioe);
}
