# Fontus Bitstream
The Fontus bit stream is broken up in to 64 bit packets that fall in to 10 different possible `MessageType`s.
Each `MessageType` is broken up in to `bitfeild`s, with the first `bitfeild` being a `MessageTypefeild` left justified in the 64-bit packet as much as possible and the last of the `bitfeild`s being a `CRCfeild` right justified in the 64-bit packet as much as much as possible.

## `bitfeild`
Is a class that contains the length of the `bitfeild` in bits as well as the necessary functions to convert to and from desired data formats and the `bitfeild` representation

## `MessageType`
Is an enum of the 10 possible types of 64-bit packages we could receive:
 - Localization Request
 - Localization Report
 - Release Request
 - Release Report
 - Lost Device Request
 - Lost Device Report
 - Status Request
 - Status Report
 - Vendor Request
 - Vendor Report
 
```
const uint64_t fromDestPort(uint64_t dest){
    // first check if valid destination
    // log error if invalid
    // second check if valid destination
    // log error if invalid
    // third check if valid destination
    // log error if invalid
    // return result
}

const uint64_t toDestPort(uint64_t bitdest){
    // conversion logic 
    // return result
}
Class Localization_Request:
uint64_t bitstream;
const uint64_t setDestPort(uint64_t dest){
    this.bitstream|=DESTPORTMASK<<LOCALIZATION_REQUEST_DEST_PORT_SHIFT;
    this.bitstream&=fromDestPort(dest)<<LOCALIZATION_REQUEST_DEST_PORT_SHIFT;
}
const uint64_t getDestPort(){
    uint64_t result=0;
    result=toDestPort((bitstream & (DESTPORTMASK<<LOCALIZATION_REQUEST_DEST_PORT_SHIFT))>>LOCALIZATION_REQUEST_DEST_PORT_SHIFT);
    return result
}
```
```
enum feilds{
    Message_Type
    Device_ID
    Date_Time
    Response_ID
    Response_Delay
    Device_Depth
    Battery
    Temperature
    Orientation
    Release_Status
    Activation_Status
    CRC
    NUM_OF_FEILDS
}
enum messageTypes{
   LOCALIZATION_REQUEST
   LOCALIZATION_REPORT
   RELEASE_REQUEST
   RELEASE_REPORT
   LOST_DEVICE_REQUEST
   Lost_Device_Report
   Status_Request
   Status_Report
   Vendor_Request
   Vendor_Report
   NUM_OF_MESSAGES
}
fn ConvFeildFns = [toMessage_Type] 

// Conversion logic Functions
int fromMessage_Type(uint64_t message_type, void* result) {
     result=message_type;
}

int fromDevice_ID((uint64_t device_id, void* result) {
    result = device_id;
}

int fromDate_Time(uint32_t a, uint32_t b) {
    return a * b;
}

// Define the function pointer type
typedef int (*ConvFunctionPtr)(uint64_t, void*);

// Array of function pointers
ConvFunctionPtr convfunctions[NUM_OF_FEILDS]; = { fromMessage_Type, fromDevice_ID, fromDate_Time, ... };

typedef struct bitfeild{
    FeildName:string
    feildValue:uint64_t
    bitmask:uint64_t
    toFn:fnptr
    fromFn:fnptr
    description:string
}

typedef struct messagetype{
    messageName:string
    messageValue:uint64_t
    description:string
    bool validfeilds[NUM_OF_FEILDS]
    uint8_t shifts[NUM_OF_FEILDS]
}

constexpr auto LocalizationRequestFeilds() {
    bool arr[NUM_OF_FEILDS] = {0}; // All elements set to 0 initially
    arr[Message_Type] = 1; // Message_Type is valid
    arr[Destination_Port] = 1; // Destination_Port is valid
    arr[Date_Time] = 1; // Date_Time is valid
    arr[CRC] = 1; // CRC is valid
    return arr;
}
constexpr auto LocalizationRequestShifts() {
    uint8_t arr[NUM_OF_FEILDS] = {0}; // All elements set to 0 initially
    arr[Message_Type] = 0; // Message_Type is valid
    arr[Destination_Port] = 6; // Destination_Port is valid
    arr[Date_Time] = 31; // Date_Time is valid
    arr[CRC] = 55; // CRC is valid
    return arr;
}

constexpr auto LocalizationResponseFeilds() {
    bool arr[NUM_OF_FEILDS] = {0}; // All elements set to 0 initially
    arr[Message_Type] = 1; // Message_Type is valid
    arr[RESPONSE_ID] = 1; // RESPONSE_ID is valid
    arr[RESPONSE_DELAY] = 1; // RESPONSE_DELAY is valid
    arr[CRC] = 1; // CRC is valid
    return arr;
}
constexpr auto LocalizationResponseShifts() {
    uint8_t arr[NUM_OF_FEILDS] = {0}; // All elements set to 0 initially
    arr[Message_Type] = 0; // Message_Type is valid
    arr[Destination_Port] = 6; // Destination_Port is valid
    arr[Date_Time] = 31; // Date_Time is valid
    arr[CRC] = 55; // CRC is valid
    return arr;
}
messagetype messages[NUM_OF_MESSAGES] = {
    {"Localization_Request",LOCALIZATION_REQUEST,"is a message to request the general localization of the device",LocalizationRequestFeilds,}
}

int64_t get(,FeildOfInterest,void *returnvalue){
// lookup in conversion function table the conversion function for the FeildOfInterest
// look up the bit mask for the FeildOfInterest in the bitmask table
// look up the bit shift for the FeildOfInterest given the MessageType
// 
}

enum FONTUS_GET_ERROR{
    MESSAGE_TYPE_HAS_NO_FEILD
}
```
```

```