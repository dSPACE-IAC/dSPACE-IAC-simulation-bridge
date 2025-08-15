import re
file_name = "dbc_structure"

def preprocess_lines(lines):
    combined_lines = []
    buffer = ""

    for line in lines:
        stripped = line.strip()
        if stripped.startswith("BO_"):
            if buffer:
                combined_lines.append(buffer)
                buffer = ""
            combined_lines.append(stripped)
        elif stripped.startswith("SG_"):
            if buffer:
                combined_lines.append(buffer)
            buffer = stripped
        elif buffer:
            buffer += " " + stripped
        else:
            combined_lines.append(stripped)

    if buffer:
        combined_lines.append(buffer)

    return combined_lines

def parse_dbc(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()

    messages = {}
    current_message = None

    for line in preprocess_lines(lines):
        message_match = re.match(r'^BO_\s+(\d+)\s+(\w+):\s+(\d+)\s+(\w+)', line)
        if message_match:
            message_id = int(message_match.group(1))
            message_name = message_match.group(2)
            dlc = int(message_match.group(3))
            transmitter = message_match.group(4)
            current_message = {
                'id': message_id,
                'name': message_name,
                'dlc': dlc,
                'transmitter': transmitter,
                'signals': []
            }
            messages[message_id] = current_message
            continue

        signal_match = re.match(
            r'^\s*SG_\s+(\w+)\s*:\s*(\d+)\|(\d+)@(\d+)([+-])\s+\(([-\d.e]+),([-\d.e]+)\)\s+\[([-\d.e]+)\|([-\d.e]+)\]\s+"(.*?)"\s+(\w+)',
            line
        )
        # print(signal_match)
        # print(current_message)
        # print("Juhu")

        if signal_match and current_message:
            # print("Juhu")
            signal = {
                'name': signal_match.group(1),
                'start_bit': int(signal_match.group(2)),
                'length': int(signal_match.group(3)),
                'endian': int(signal_match.group(4)),
                'is_signed': signal_match.group(5),
                'factor': float(signal_match.group(6)),
                'offset': float(signal_match.group(7)),
                'min_val': float(signal_match.group(8)),
                'max_val': float(signal_match.group(9)),
                'unit': signal_match.group(10),
                'receiver': signal_match.group(11)
            }
            current_message['signals'].append(signal)

    return messages

def determine_cpp_type(signal):
    length = signal['length']
    signed = signal['value_type'] == '-'
    if length <= 8:
        return 'int8_t' if signed else 'uint8_t'
    elif length <= 16:
        return 'int16_t' if signed else 'uint16_t'
    elif length <= 32:
        return 'int32_t' if signed else 'uint32_t'
    else:
        return 'int64_t' if signed else 'uint64_t'

def generate_header(messages, output_header):
    with open(output_header, 'w') as file:
        file.write(f'#ifndef {file_name.upper()}_H\n#define {file_name.upper()}_H\n\n#include <cstdint>\n#include <vector>\n#include <cstddef>\n\n')
        file.write('struct Signal {\n')
        file.write('    const char* name;\n')
        file.write('    uint8_t start_bit;\n')
        file.write('    uint8_t length;\n')
        file.write('    uint8_t endian;\n')
        file.write('    bool is_signed;\n')
        file.write('    float factor;\n')
        file.write('    float offset;\n')
        file.write('    float min_val;\n')
        file.write('    float max_val;\n')
        file.write('    const char* unit;\n')
        file.write('};\n\n')
        file.write('struct Message {\n')
        file.write('    uint32_t id;\n')
        file.write('    uint8_t dlc;\n')
        file.write('    const char* name;\n')
        file.write('    const char* transmitter;\n')
        file.write('    std::vector<Signal> signals;\n')
        # file.write('    Signal* signals;\n')
        file.write('    size_t signal_count;\n')
        file.write('};\n\n')
        file.write('extern std::vector<Message> initialize_messages();\n')
        # file.write('extern Message* initialize_messages();\n')
        file.write(f'#endif // {file_name.upper()}_H\n')

def generate_source(messages, output_source):
    with open(output_source, 'w') as file:
        file.write(f'#include "{file_name}.h"\n\n')
        file.write('#include <cstdlib>\n\n')
        # file.write('Message* initialize_messages() {\n')
        file.write('std::vector<Message> initialize_messages() {\n')
        file.write(f'    size_t message_count = {len(messages)};\n')
        # file.write('    Message* messages = (Message*)malloc(sizeof(Message) * (message_count));\n\n')
        file.write('    std::vector<Message> messages(message_count);\n\n')
        for i, message in enumerate(messages.values()):
            file.write(f'    messages[{i}].id = {message["id"]};\n')
            file.write(f'    messages[{i}].dlc = {message["dlc"]};\n')
            file.write(f'    messages[{i}].name = "{message["name"]}";\n')
            file.write(f'    messages[{i}].transmitter = "{message["transmitter"]}";\n')
            file.write(f'    messages[{i}].signal_count = {len(message["signals"])};\n')
            # file.write(f'    messages[{i}].signals = (Signal*)malloc(sizeof(Signal) * messages[{i}].signal_count);\n')
            file.write(f'    messages[{i}].signals.resize(messages[{i}].signal_count);\n')
            for j, signal in enumerate(message['signals']):
                file.write(f'    messages[{i}].signals[{j}] = Signal{{ \"{signal["name"]}\", {signal["start_bit"]}, {signal["length"]}, {signal["endian"]}, {"false" if signal["is_signed"] == "+" else "true"}, {signal["factor"]}f, {signal["offset"]}f, {signal["min_val"]}f, {signal["max_val"]}f, "{signal["unit"]}" }};\n')
            file.write('\n')
        file.write('    return messages;\n')
        file.write('}\n')

if __name__ == "__main__":
    dbc_file = "CAN1-INDY-V23.dbc"
    output_header = f"./../include/{file_name}.h"
    # output_header = f"{file_name}.h"
    output_source = f"./../src/{file_name}_init.cpp"
    # output_source = f"{file_name}_init.cpp"
    messages = parse_dbc(dbc_file)
    generate_header(messages, output_header)
    generate_source(messages, output_source)
    print(f"Header file '{output_header}' and source file '{output_source}' generated successfully.")
