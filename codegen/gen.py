import yaml
import random

# Template for the header of the C++ file
hpp_header = """/***************************************************/
/********** GENERATED FILE, DO NOT EDIT ************/
/***************************************************/

#ifndef GENERATED_BITSTREAM_HPP
#define GENERATED_BITSTREAM_HPP


// Define the BitstreamParameter structure
typedef struct {
    std::string name;    // Name of the field
    int startbit;        // Start of field
    int bitDepth;        // Field size in bits
} BitstreamParameter;

// Assume FAMmessages and other tables are defined here
extern const BitstreamParameter* FAMmessages[64]; // Defined elsewhere in flash

"""

hpp_getField_auto_detect = """
#include "RequestStatusEnum.hpp"

int32_t getMessageType(uint64_t *bitstream);
void setField(uint64_t *bitstream, 
              const char *ParameterName, 
              uint64_t new_value,
              RequestStatus *status) ;

// Function to extract field from a bitstream
uint64_t getField(uint64_t *bitstream, 
                  const char *ParameterName,
                  RequestStatus *status) ;

"""
# Updated getField function template (auto-detects the message type)
cpp_getField_auto_detect = """/***************************************************/
/********** GENERATED FILE, DO NOT EDIT ************/
/***************************************************/


//#include "RequestStatusEnum.hpp"
#include "generated_bitstream.hpp"
#include <iostream>
#include <cstring> // For strcmp
#include <cstdint> // For int64_t, int32_t, etc.

int32_t getMessageType(uint64_t *bitstream){
    uint64_t bitmask = 0xFC00000000000000;
    uint64_t shiftval = 58;
    uint64_t masked_type = (*bitstream & bitmask); 
    uint64_t type =  masked_type >> shiftval; 
    return type;
}
void setField(uint64_t *bitstream, 
              const char *ParameterName, 
              uint64_t new_value,
              RequestStatus *status) {
    // Step 1: Extract the message type from the bitstream
    int32_t message_type = getMessageType((uint64_t*) bitstream);
    const BitstreamParameter *message=nullptr;
    int startbit;
    int bitDepth;
    uint64_t mask;

    // Validate the message_type
    if (ParameterName == "Type"){
        message = FAMmessages[new_value];
        mask = (1ULL << 6)-1;
        uint64_t bit_offset = 58;
        // Step 3: Clear the bits where the field resides
        uint64_t shifted_mask = mask << bit_offset;  // Mask for the field
        *bitstream &= ~shifted_mask;  // Clear the relevant bits

        // Step 4: Set the new value in the bitstream
        *bitstream |= (static_cast<uint64_t>(new_value) << bit_offset);  // Set new value

        *status = REQUEST_VALID;
        return;  // Exit after setting the field
    }else if ((message_type < 0) || (message_type >= 64) || (FAMmessages[message_type] == 0/*nullptr*/)) {
        std::cerr << "Invalid message_type or uninitialized." << std::endl;
        *status = INVALID_MESSAGE_TYPE;
        return;
    }else{
        message = FAMmessages[message_type];
        ///const BitstreamParameter *message = FAMmessages[new_value];
    }

    int i = 0;
    // Step 2: Search for the ParameterName in the specified message
    if(message==nullptr){
        *status = INVALID_MESSAGE_TYPE;
        return;
    }
    for (i = 0; message[i].bitDepth > 0; ++i) {
        if (strcmp(message[i].name.c_str(), ParameterName) == 0) {
            startbit = message[i].startbit;
            bitDepth = message[i].bitDepth;

            // Ensure that the new_value fits within the bitDepth of the field
            if (new_value >= (1ULL << bitDepth)) {
                std::cerr << "Error: Value too large for bit depth." << std::endl;
                *status = VALUE_TOO_LARGE_FOR_BITDEPTH;
                return;
            }

            // Step 3: Clear the bits where the field resides
            int bit_offset = startbit;
            mask = ((1ULL << bitDepth) - 1) << bit_offset;  // Mask for the field
            *bitstream &= ~mask;  // Clear the relevant bits

            // Step 4: Set the new value in the bitstream
            *bitstream |= (static_cast<uint64_t>(new_value) << bit_offset);  // Set new value

            *status = REQUEST_VALID;
            return;  // Exit after setting the field
        }
    }

    // Step 5: Print an error if ParameterName is not found
    std::cerr << "Error: Parameter name not found." << std::endl;
    *status = FIELD_NOT_IN_MESSAGE;
}

// Function to extract field from a bitstream
uint64_t getField(uint64_t *bitstream, 
                  const char *ParameterName,
                  RequestStatus *status) {
  {
    int32_t message_type = getMessageType((uint64_t*) bitstream);
    uint64_t bit_offset;
    uint64_t mask;
    int startbit;
    int bitDepth;
    const BitstreamParameter *message;
    // Step 1: Validate the message_type
    if (ParameterName == "Type"){
        mask = (1ULL<<6)-1;
        bit_offset = 58;
        // Extract the relevant bits from the bitstream
        uint64_t value = (*bitstream >> bit_offset);

        // Mask out extra bits beyond the bitDepth and return the value
        *status = REQUEST_VALID;
        return value & mask;
    }else if ((message_type < 0) || (message_type >= 64) ||
        (FAMmessages[message_type] == nullptr)) {
      {
        *status = INVALID_MESSAGE_TYPE; // Invalid message_type or uninitialized
        return 0ULL;
      }
    }

    message = FAMmessages[message_type];

    int i = 0;
    // Step 2: Search for the ParameterName in the specified message
    for (i = 0; message[i].bitDepth > 0; ++i) {
      {
        if (strcmp(message[i].name.c_str(), ParameterName) == 0) {
          {
            // Step 3: Extract the field from the bitstream
            startbit = message[i].startbit;
            bitDepth = message[i].bitDepth;

            // Calculate the bitDepth
            int bit_offset = startbit % 64;

            // Extract the relevant bits from the bitstream
            uint64_t value = (*bitstream >> bit_offset);

            // Mask out extra bits beyond the bitDepth and return the value
            uint64_t mask = (1UL << bitDepth) - 1;
            *status = REQUEST_VALID;
            return value & mask;
          }
        }
      }
    }

    // Step 4: Return 0 if ParameterName is not found
    *status = FIELD_NOT_IN_MESSAGE;
    return 0;
  }
}


"""


# Function to generate C++ code for a message
def generate_message(name, fields):
    fontusmessagesize = 64
    cpp_message = f"// {name} Table\n"
    cpp_message += f"const BitstreamParameter {name}[] = {{\n"

    startbit = 0
    for field in fields:
        for field_name, attributes in field.items():
            bit_size = attributes[0]["size"]
            cpp_message += f'    {{"{field_name}", {fontusmessagesize-startbit-bit_size}, {bit_size}}},\n'
            startbit += bit_size

    cpp_message += "};\n\n"
    return cpp_message


# Function to create the FAMmessages array
def generate_fammessages(messages, max_size=64):
    cpp_fammessages = "const BitstreamParameter* FAMmessages[64] = {\n"
    fam_array = ["nullptr"] * max_size  # Initialize array with nullptrs

    for message_name, index in messages.items():
        fam_array[index] = message_name

    for msg in fam_array:
        cpp_fammessages += f"    {msg},\n"

    cpp_fammessages += "};\n\n"
    return cpp_fammessages

def generate_random_failing_tests(message_indices,data,tests_per_field):
    gtest_code = """TEST(BitstreamTest, generated_setval_getvaltest_failing){
        uint64_t bitfield[3] = {0xFC00000000000000, 0x0000000000000000, 0xFC00000000000000};
        uint64_t bitfield_val;
        RequestStatus status=REQUEST_VALID;
    """
    random.seed(567)
    for numberofcompletefieldtests in range(0,tests_per_field):
        for message_name, fields in data.items():
            for field in fields:
                for field_name, attributes in field.items():
                    bit_size = attributes[0]["size"]
                    bit_mask = (1 << bit_size) -1
                    gtest_code += f"""
    // Testing {message_name} message with the {field_name} field being typed incorrectly
    bitfield_val = {random.randint(0,bit_mask)};
    setField(&bitfield[1], "Type", {message_indices[message_name]},&status); // assign the type we are working with so the field we are testing is valid

    // the status should always report a valid request on this function call
    EXPECT_EQ(status, REQUEST_VALID);

    setField(&bitfield[1], "not_{field_name}", bitfield_val,&status);
    // the status should always report a valid request on these function calls
    EXPECT_EQ(status, FIELD_NOT_IN_MESSAGE);

    getField(&bitfield[1], "not_{field_name}",&status);

    // the status should always report a valid request on these function calls
    EXPECT_NE(status, REQUEST_VALID);
    
    // return bitfield to be all zeroes
    setField(&bitfield[1], "Type", 0,&status);

    // ensuring we don't contaminate the previous bitfield or next bitfield
    EXPECT_EQ(bitfield[0], 0xFC00000000000000);
    EXPECT_EQ(bitfield[1], 0x0000000000000000);
    EXPECT_EQ(bitfield[2], 0xFC00000000000000);

"""
    gtest_code += "}\r\n"
    return gtest_code

def generate_random_passing_tests(message_indices,data,tests_per_field):
    gtest_code = """TEST(BitstreamTest, generated_setval_getvaltest_passing){
        uint64_t bitfield[3] = {0xFC00000000000000, 0xFC00000000000000, 0xFC00000000000000};
        uint64_t bitfield_val;
        RequestStatus status=REQUEST_VALID;
    """
    random.seed(567)
    for numberofcompletefieldtests in range(0,tests_per_field):
        for message_name, fields in data.items():
            for field in fields:
                for field_name, attributes in field.items():
                    bit_size = attributes[0]["size"]
                    bit_mask = (1 << bit_size) -1
                    gtest_code += f"""
    // Testing {message_name} message with the {field_name} field being set
    bitfield_val = {random.randint(0,bit_mask)};
    setField(&bitfield[1], "Type", {message_indices[message_name]},&status); // assign the type we are working with so the field we are testing is valid

    // the status should always report a valid request on these function calls
    EXPECT_EQ(status, REQUEST_VALID);

    setField(&bitfield[1], "{field_name}", bitfield_val,&status);
    // the status should always report a valid request on these function calls
    EXPECT_EQ(status, REQUEST_VALID);

    EXPECT_EQ(getField(&bitfield[1], "{field_name}",&status), bitfield_val);

    // the status should always report a valid request on these function calls
    EXPECT_EQ(status, REQUEST_VALID);
    
    // ensuring we don't contaminate the previous bitfield or next bitfield
    EXPECT_EQ(bitfield[0], 0xFC00000000000000);
    EXPECT_EQ(bitfield[2], 0xFC00000000000000);

"""
    gtest_code += "}\r\n"
    return gtest_code

def gtest_code(gtest_file,yaml_file,hpp_file):
    message_indices = {}
    with open(yaml_file, "r") as file:
        data = yaml.safe_load(file)

    # Generate C++ code for each message in the YAML
    for message_name, fields in data.items():
        for field in fields:
            if "Type" in field and "value" in field["Type"][0]:
                value = field["Type"][0]["value"]
                message_indices[message_name] = value
                break
    gtest_code = f"""/***************************************************/
/********** GENERATED FILE, DO NOT EDIT ************/
/***************************************************/

#include <gtest/gtest.h>
#include "{hpp_file}" // Include your code with getField function

// Test the getMessageType function for various Types
TEST(BitstreamTest, GetMessageTypeTest) {{
    uint64_t bitfield = 0xFFFFFFFFFFFFFFFF;
    EXPECT_EQ(getMessageType(&bitfield), 0x3F);

    bitfield = 0x0000FFFFFFFFFFFF;
    EXPECT_EQ(getMessageType(&bitfield), 0);
    
    bitfield = 0x0400FFFFFFFFFFFF;
    EXPECT_EQ(getMessageType(&bitfield), 1);
    
    bitfield = 0x0800FFFFFFFFFFFF;
    EXPECT_EQ(getMessageType(&bitfield), 2);
    
    bitfield = 0x0C00FFFFFFFFFFFF;
    EXPECT_EQ(getMessageType(&bitfield), 3);
    
    bitfield = 0xF800FFFFFFFFFFFF;
    EXPECT_EQ(getMessageType(&bitfield), 62);
}}

// Test the getField function for various scenarios
TEST(BitstreamTest, GetFieldTest) {{
    uint64_t bitfield = 0xFFFFFFFFFFFFFFFF;
    RequestStatus status=REQUEST_VALID;
    
    // Test CRC extraction
    EXPECT_NE(getField(&bitfield, "CRC",&status), 0xFFF);
    EXPECT_EQ(status, INVALID_MESSAGE_TYPE);

    // Test invalid field extraction
    EXPECT_EQ(getField(&bitfield, "InvalidField",&status), 0);
    EXPECT_EQ(status, INVALID_MESSAGE_TYPE);

    // Test Type extraction (LEFT MOST 6 bits)
    bitfield = 0x400000000000000; // Type = 1
    EXPECT_EQ(getField(&bitfield, "Type",&status), 1);

    // Test Type extraction (LEFT MOST 6 bits)
    bitfield = 0x800000000000000; // Type = 2
    EXPECT_EQ(getField(&bitfield, "Type",&status), 2);

    // Test Type extraction (LEFT MOST 6 bits)
    bitfield = 0xC00000000000000; // Type = 3
    EXPECT_EQ(getField(&bitfield, "Type",&status), 3);

    // Test Type extraction (LEFT MOST 6 bits)
    bitfield = 0x700000000000000; // Type = 1
    EXPECT_EQ(getField(&bitfield, "Type",&status), 1);

    // Test Type extraction (LEFT MOST 6 bits)
    bitfield = 0xB00000000000000; // Type = 2
    EXPECT_EQ(getField(&bitfield, "Type",&status), 2);

    // Test Type extraction (LEFT MOST 6 bits)
    bitfield = 0xF00000000000000; // Type = 3
    EXPECT_EQ(getField(&bitfield, "Type",&status), 3);

    // Test Type extraction (LEFT MOST 6 bits)
    bitfield = 0xFFFFFFFFFFFFFFF; // Type = 3
    EXPECT_EQ(getField(&bitfield, "Type",&status), 3);

    //// // Test another value (set Device_ID)
    //// bitfield = 0x0000000F00000000; // Device_ID = 0xF
    //// EXPECT_EQ(getField(&bitfield, "Device_ID",&status), 0xF);
}}

TEST(BitstreamTest, setval_getvaltest){{
    uint64_t bitfield = 0xFFFFFFFFFFFFFFFF;
    RequestStatus status=REQUEST_VALID;

    // type = 5
    uint64_t bitfield_val = 5;
    setField(&bitfield, "Type", bitfield_val,&status);
    EXPECT_EQ(status, REQUEST_VALID);
    EXPECT_EQ(getField(&bitfield, "Type",&status), bitfield_val);
    EXPECT_EQ(status, REQUEST_VALID);

    bitfield_val = 63;
    setField(&bitfield, "Type", bitfield_val,&status);
    EXPECT_EQ(getField(&bitfield, "Type",&status), bitfield_val);

    bitfield_val = 6;
    setField(&bitfield, "Type", bitfield_val,&status);
    EXPECT_EQ(getField(&bitfield, "Type",&status), bitfield_val);
    
    bitfield_val = 3;
    setField(&bitfield, "CRC", bitfield_val,&status);
    EXPECT_EQ(getField(&bitfield, "CRC",&status), bitfield_val);

    bitfield_val = 4;
    setField(&bitfield, "CRC", bitfield_val,&status);
    EXPECT_EQ(getField(&bitfield, "CRC",&status), bitfield_val);

    bitfield_val = 431;
    setField(&bitfield, "CRC", bitfield_val,&status);
    EXPECT_EQ(getField(&bitfield, "CRC",&status), bitfield_val);
    
}}

TEST(BitstreamTest, invalidsetval_getvaltest){{
    uint64_t bitfield = 0x0000000000000000;
    RequestStatus status=REQUEST_VALID;

    // Incorrect Type set commands
    uint64_t bitfield_val = 5;
    setField(&bitfield, "TYpe", bitfield_val,&status);
    EXPECT_NE(status, REQUEST_VALID);
    EXPECT_NE(getField(&bitfield, "TYpe",&status), bitfield_val);
    EXPECT_NE(status, REQUEST_VALID);

    bitfield_val = 6;
    setField(&bitfield, "TyPe", bitfield_val,&status);
    EXPECT_NE(status, REQUEST_VALID);
    EXPECT_NE(getField(&bitfield, "Type",&status), bitfield_val);

    bitfield_val = 63;
    setField(&bitfield, "TypE", bitfield_val,&status);
    EXPECT_NE(status, REQUEST_VALID);
    EXPECT_NE(getField(&bitfield, "Type",&status), bitfield_val);

    // Incorrect CRC set commands
    bitfield_val = 3;
    setField(&bitfield, "cRC", bitfield_val,&status);
    EXPECT_NE(status, REQUEST_VALID);
    EXPECT_NE(getField(&bitfield, "CRC",&status), bitfield_val);

    bitfield_val = 4;
    setField(&bitfield, "CrC", bitfield_val,&status);
    EXPECT_NE(status, REQUEST_VALID);
    EXPECT_NE(getField(&bitfield, "CRC",&status), bitfield_val);

    bitfield_val = 431;
    setField(&bitfield, "CRc", bitfield_val,&status);
    EXPECT_NE(status, REQUEST_VALID);
    EXPECT_NE(getField(&bitfield, "CRC",&status), bitfield_val);
    
}}
"""
    gtest_code += generate_random_passing_tests(message_indices,data,2)
    gtest_code += generate_random_failing_tests(message_indices,data,2)

    gtest_code +=f"""
int main(int argc, char **argv) {{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}}
"""
    with open(gtest_file, "w") as file:
        file.write(gtest_code)


# Main function to convert YAML to C++ file
def yaml_to_cpp(yaml_file, cpp_file):
    with open(yaml_file, "r") as file:
        data = yaml.safe_load(file)

    cpp_code = cpp_header
    message_indices = {}

    # Generate C++ code for each message in the YAML
    for message_name, fields in data.items():
        for field in fields:
            if "Type" in field and "value" in field["Type"][0]:
                value = field["Type"][0]["value"]
                message_indices[message_name] = value
                break

        # Append generated message code to hpp_code
        cpp_code += generate_message(message_name, fields)

    cpp_code += generate_fammessages(message_indices)

    # Append the auto-detect getField and setField function
    cpp_code += hpp_getField_auto_detect
    cpp_code += "\r\n#endif //GENERATED_BITSTREAM_HPP"

    with open(cpp_file, "w") as file:
        file.write(cpp_code)


# Example usage
yaml_file = "bitstream.yml"  # Input YAML file
cpp_file = "../src/generated_bitstream.cpp"  # Output C++ file
hpp_file = "../Common/nfwf_include/generated_bitstream.hpp"  # Output C++ header file
gtest_file = "../test/gtest_bitstream.cpp"  # Output C++ header file
yaml_to_cpp(yaml_file, cpp_file)
print(f"C++ header file '{cpp_file}' has been generated from '{yaml_file}'.")

gtest_code(gtest_file, yaml_file, hpp_file)
print(f"C++ gtest file '{gtest_file}' has been generated from '{yaml_file}'.")