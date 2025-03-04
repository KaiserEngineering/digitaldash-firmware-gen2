import json
from datetime import date

TotalByteCount = 0
EEPROM_Count = 0
PageCount = 0

today = date.today()

code_header = open('header.h', 'r').read()
code_header = code_header.replace("<today>", today.strftime("%b-%d-%Y"))
code_header = code_header.replace("<year>", today.strftime("%Y"))

# Open the command list
with open('config.json') as f:
  config = json.load(f)

print("\n---- GENERATING SETTINGS ----\n")

max_views = config["config"]["max_views"]
print("Number of views: " + str(max_views))

gauges_per_view = config["config"]["gauges_per_view"]
print("Max gauges per view: " + str(gauges_per_view))

max_alerts = config["config"]["max_alerts"]
print("Max Alerts: " + str(max_alerts))

alert_message_len = config["config"]["alert_message_len"]
print("Alert_Message_Len: " + str(alert_message_len))

top_level_struc = config["config"]["top_level_struc"]
top_level_name = config["config"]["top_level_name"]

# Create the c file
config_c = open("../Core/Src/ke_config.c", "w")
config_h = open("../Core/Inc/ke_config.h", "w")

config_c.write( code_header + "\n\n" )
config_h.write( code_header + "\n\n" )

config_h.write("#ifndef KE_CONFIG_H\n")
config_h.write("#define KE_CONFIG_H\n\n")
config_h.write("#ifdef __cplusplus\n")
config_h.write("extern \"C\"\n{\n")
config_h.write("#endif\n\n")

config_h.write("#include \"lvgl.h\"\n")
config_h.write("#include \"stdbool.h\"\n\n")

config_h.write("typedef void(settings_write)(uint16_t bAdd, uint8_t bData);\n")
config_h.write("typedef uint8_t(settings_read)(uint16_t bAdd);\n\n")

config_h.write("void settings_setWriteHandler(settings_write *writeHandler);\n")
config_h.write("void settings_setReadHandler(settings_read *readHandler);\n\n")

config_h.write("#define GAUGES_PER_VIEW " + str(gauges_per_view) + "\n")
config_h.write("#define MAX_ALERTS " + str(max_alerts) + "\n")
config_h.write("#define ALERT_MESSAGE_LEN " + str(alert_message_len)+ "\n")
config_h.write("#define MAX_VIEWS " + str(max_views)+ "")

config_c.write( "#include \"ke_config.h\"\n\n" )
#config_c.write( "static " + top_level_struc + " " + top_level_name + ";\n\n")

def get_eeprom_size(cmd):
   if isinstance(cmd["EEBytes"], int):
      return cmd["EEBytes"]
   else:
      return config["config"][cmd["EEBytes"]]

def write_default_define(file, prefix, cmd, depth):
   print("[ADDED] " + cmd["name"])
   if( str(cmd["default"]).isnumeric()):
     file.write("#define DEFAULT_" + prefix.upper() + "_" + cmd["cmd"].upper() + " " + str(cmd["default"]) + "\n")
   else:
     if( cmd["type"] == "string" ):
        file.write("#define DEFAULT_" + prefix.upper() + "_" + cmd["cmd"].upper() + " " + cmd["default"] + "\n")
     else:
        file.write("#define DEFAULT_" + prefix.upper() + "_" + cmd["cmd"].upper() + " " + cmd["dataType"].replace(" ", "_").upper() + "_" + cmd["default"].replace(" ", "_").upper() + "\n")

def write_comment_block( file, prefix, cmd, depth ):
    file.write("\n\n")
    file.write("/********************************************************************************\n")
    file.write("*" + cmd["name"].center(79) + "\n")
    file.write("*\n")
    if cmd["index"]:
      if depth == 2:
        file.write("* @param idx_" + prefix.split('_')[0].lower() + "    index of the " + prefix.split('_')[0] + "\n")
        file.write("* @param idx_" + prefix.split('_')[1].lower() + "    index of the " + prefix.split('_')[1] + "\n")
      else:
         file.write("* @param idx_" + prefix.lower() + "    index of the " + prefix + "\n")
    file.write("* @param " + cmd["cmd"] + "    " + cmd["desc"] + "\n")
    file.write("* @param save    Set true to save to the EEPROM, otherwise value is non-volatile\n")
    file.write("*\n")
    file.write("********************************************************************************/\n")

def write_custom_struct( file, prefix, cmd, depth ):
  if cmd["type"] == "list":
    file.write("typedef enum\n{\n")
    for enum in cmd["options"]:
        file.write("    " + cmd["dataType"] + "_" + enum.replace(" ", "_").upper() + ",\n")
    file.write("    " + cmd["dataType"] + "_" + cmd["limit"].replace(" ", "_").upper() + "\n")
    file.write("} " + cmd["dataType"] + ";\n\n")

def write_verify_declare( file, prefix, cmd, depth ):
    file.write( "bool verify_" + prefix + "_" + cmd["cmd"].lower() + "(" + cmd["dataType"] + " " + cmd["cmd"].lower() + ");\n" )

def write_verify_source( file, prefix, cmd, depth ):
    input = cmd["cmd"].lower()
    file.write( "bool verify_" + prefix + "_" + cmd["cmd"].lower() + "(" + cmd["dataType"] + " " + input + ")\n" )
    file.write( "{\n" )
    if cmd["type"] == "number" or cmd["type"] == "slider":
      # Only check for less than 0 if the datatype is an integer
      if( not((cmd["dataType"].find("uint") >= 0) and ( cmd["min"] == 0 )) ):
          file.write("    if (" + input + " < " +  str(cmd["min"]) + ")\n" )
          file.write("        return 0;\n\n" )

      if( not((cmd["max"] == 255) and (cmd["dataType"] == "uint8_t")) ):
          file.write("    if (" + input + " > " +  str(cmd["max"]) + ")\n" )
          file.write("        return 0;\n\n" )
          file.write("    else\n" )
          file.write("        return 1;" )
      else:
          file.write("    return 1;" )

    elif cmd["type"] == "list":
        file.write("    if (" + input + " >= " + cmd["dataType"] + "_" + cmd["limit"].replace(" ", "_").upper() + ")\n" )
        file.write("        return 0;\n" )

        file.write("    else\n" )
        file.write("        return 1;" )

    elif cmd["type"] == "pointer":
        file.write("    if (" + input + " != " +  str(cmd["limit"]) + ")\n" )
        file.write("        return 1;" )

    elif cmd["type"] == "string":
        file.write("    return 1; // TODO - String checking" )
       

    file.write("\n}\n\n")

def write_get_declare( file, prefix, cmd, depth ):
    # Get function definition
    if cmd["index"]:
      if( depth == 2):
        func = "uint8_t idx_" + prefix.split('_')[0].lower() + ", uint8_t idx_" + prefix.split('_')[1].lower()
      else:
         func = "uint8_t idx_" + prefix.lower()
    else:
      func = "void"
    
    file.write( cmd["dataType"] + " get_" + prefix + "_" + cmd["cmd"].lower() + "(" + func + ");\n" )

def write_set_declare( file, prefix, cmd, depth ):
    # Set function definition
    if cmd["index"]:
      if( depth == 2):
        func = "uint8_t idx_" + prefix.split('_')[0].lower() + ", uint8_t idx_" + prefix.split('_')[1].lower() + ", "
      else:
         func = "uint8_t idx_" + prefix.lower() + ","
    else:
      func = ""

    func = func + cmd["dataType"] + " " + cmd["cmd"].lower();

    file.write( "bool set_" + prefix + "_" + cmd["cmd"] + "(" + func + ", bool save);\n" )

def sub_write_memory_organization( file, prefix, cmd, idx ):
  global PageCount, EEPROM_Count, TotalByteCount
  byte_count = 1
  #define EEPROM byte offset
  while byte_count <= get_eeprom_size(cmd):
    if( idx > 0 ):
      file.write("#define EEPROM_" + prefix.upper() + "_" + cmd["cmd"].upper() + str(idx) + "_BYTE" +  str(byte_count) + " (uint16_t)" + '0x{:02X}'.format(PageCount) + '{:02X}'.format(EEPROM_Count) + "\n")
    else:
       file.write("#define EEPROM_" + prefix.upper() + "_" + cmd["cmd"].upper() + "_BYTE" +  str(byte_count) + " (uint16_t)" + '0x{:02X}'.format(PageCount) + '{:02X}'.format(EEPROM_Count) + "\n\n")
    EEPROM_Count = EEPROM_Count + 1
    TotalByteCount = TotalByteCount + 1
    if( EEPROM_Count >= 32 ):
        PageCount = PageCount + 1
        EEPROM_Count = 0
    byte_count = byte_count + 1

def sub_write_memory_organization_2d( file, prefix, cmd, idx1, idx2 ):
  global PageCount, EEPROM_Count, TotalByteCount
  byte_count = 1
  #define EEPROM byte offset
  while byte_count <= get_eeprom_size(cmd):
    file.write("#define EEPROM_" + prefix.upper().split('_')[0].upper() + str(idx1) + "_" + prefix.upper().split('_')[1].upper() + "_" + cmd["cmd"].upper() + str(idx2) + "_BYTE" +  str(byte_count) + " (uint16_t)" + '0x{:02X}'.format(PageCount) + '{:02X}'.format(EEPROM_Count) + "\n")
    EEPROM_Count = EEPROM_Count + 1
    TotalByteCount = TotalByteCount + 1
    if( EEPROM_Count >= 32 ):
        PageCount = PageCount + 1
        EEPROM_Count = 0
    byte_count = byte_count + 1

def sub_write_memory_map( file, prefix, cmd, byte, depth ):
   if( depth == 2 ):
      for i in range(config["config"][cmd["count"][0]]):
         array = "{ "
         for j in range(config["config"][cmd["count"][1]]):
            array = array + "EEPROM_" + prefix.upper().split('_')[0].upper() + str(i+1) + "_" + prefix.upper().split('_')[1].upper() + "_" + cmd["cmd"].upper() + str(j+1) + "_BYTE" +  str(byte)
            if( j < config["config"][cmd["count"][1]]):
               array = array +  ", "
         array = array + " }"
         file.write("#define EEPROM_" + prefix.upper().split('_')[0].upper() + str(i+1) + "_" + prefix.upper().split('_')[1].upper() + "_" + cmd["cmd"].upper() + "_BYTE" + str(byte) + array + "\n")
      file.write("static const uint16_t map_" + prefix.lower() + "_" + cmd["cmd"].lower() + "_byte" + str(byte) + "[" + cmd["count"][0].upper() + "]" + "[" + cmd["count"][1].upper() + "] = {" + "\n")
      for i in range(config["config"][cmd["count"][0]]):
        file.write("    EEPROM_" + prefix.upper() + "_" + cmd["cmd"].upper() + str(i+1) + "_BYTE" + str(byte) )
        if i < (config["config"][cmd["count"][0]]-1):
          file.write(",\n")
        else:
          file.write("\n")
      file.write("    };\n\n")
   else:
    file.write("static const uint16_t map_" + prefix.lower() + "_" + cmd["cmd"].lower() + "_byte" + str(byte) + "[" + cmd["count"].upper() + "] = {" + "\n")
    for i in range(config["config"][cmd["count"]]):
        file.write("    EEPROM_" + prefix.upper() + "_" + cmd["cmd"].upper() + str(i+1) + "_BYTE" + str(byte) )
        if i < (config["config"][cmd["count"]]-1):
          file.write(",\n")
        else:
          file.write("\n")
    file.write("    };\n\n")

def write_memory_organization( file, prefix, cmd, depth ):
   if( get_eeprom_size(cmd) > 0 ):
        file.write("// EEPROM Memory Map - " + prefix + " " + cmd["cmd"] + "\n")
        if( cmd["index"] ):
           if( depth == 2 ):
              for i in range(config["config"][cmd["count"][0]]):
                for j in range(config["config"][cmd["count"][1]]):
                  sub_write_memory_organization_2d( file, prefix, cmd, i+1, j+1 )
              for byte in range(get_eeprom_size(cmd)):
                sub_write_memory_map( file, prefix, cmd, byte+1, depth )
           else:
              for i in range(config["config"][cmd["count"]]):
                sub_write_memory_organization( file, prefix, cmd, i+1 )
              for byte in range(get_eeprom_size(cmd)):
                sub_write_memory_map( file, prefix, cmd, byte+1, depth )
        else:
           sub_write_memory_organization( file, prefix, cmd, 0 )

def write_variables( file, prefix, cmd, depth ):
      append = ""
      if( cmd["type"] == "string"):
          append = "[" + cmd["EEBytes"].upper() + "]"
      if( cmd["index"] ):
        if( depth == 2 ):
          file.write( "static " + cmd["dataType"] + " " +  prefix + "_" + cmd["cmd"] +  "[" + cmd["count"][0].upper() + "]" + append + " = {DEFAULT_" + prefix.upper() + "_" + cmd["cmd"].upper() + "};\n" )
        else:
           file.write( "static " + cmd["dataType"] + " " +  prefix + "_" + cmd["cmd"] +  "[" + cmd["count"].upper() + "]" + append + " = {DEFAULT_" + prefix.upper() + "_" + cmd["cmd"].upper() + "};\n" )
      else:
        file.write( "static " + cmd["dataType"] + " " + prefix + "_" + cmd["cmd"] +  " = DEFAULT_" + prefix.upper() + "_" + cmd["cmd"].upper() + ";\n" )

def write_load_source( file, prefix, cmd, depth ):
   if cmd["index"]:
      if( depth == 2 ):
        file.write( "static " + cmd["dataType"] + " load_" + prefix.lower() + "_" + cmd["cmd"].lower() + "(uint8_t idx_" + prefix.lower().split('_')[0] + ", uint8_t idx_" + prefix.lower().split('_')[1] + ")\n")
      else:
        file.write( "static " + cmd["dataType"] + " load_" + prefix.lower() + "_" + cmd["cmd"].lower() + "(uint8_t idx)\n")
   else:
      file.write( "static " + cmd["dataType"] + " load_" + prefix.lower() + "_" + cmd["cmd"].lower() + "(void)\n")
   file.write( "{\n" )
   file.write( "    " + cmd["dataType"] + " load_" + prefix.lower() + "_" + cmd["cmd"].lower() + "_val = DEFAULT_" + prefix.upper() + "_" + cmd["cmd"].upper() + ";\n\n") 
   # Only values that are saved in EEPROM need to be added
   if( get_eeprom_size(cmd) > 0 ):
    if( cmd["index"] ):
      if( depth == 2 ):
         index = "idx_" + prefix.lower().split('_')[0] + "][idx_" + prefix.lower().split('_')[1]
      else:
         index = "idx"
      if( get_eeprom_size(cmd) == 1 ):
          file.write("    if (get_eeprom_status() == EEPROM_STATUS_PRESENT)\n    {\n");
          file.write("        load_" + prefix.lower() + "_" + cmd["cmd"].lower() + "_val = (uint8_t)read(map_" + prefix.lower() + "_" + cmd["cmd"].lower() + "_byte1[" + index + "]);\n")
          file.write("    }\n")
      elif( get_eeprom_size(cmd) == 2 ):
          file.write("    if (get_eeprom_status() == EEPROM_STATUS_PRESENT)\n    {\n");
          file.write("        load_" + prefix.lower() + "_" + cmd["cmd"].lower() + "_val = (uint16_t)read(map_" + prefix.lower() + "_" + cmd["cmd"].lower() + "_byte1[" + index + "]);\n")
          file.write("        load_" + prefix.lower() + "_" + cmd["cmd"].lower() + "_val = ((uint16_t)load_" + prefix.lower() + "_" + cmd["cmd"].lower() + "_val << 8) | " + "(uint16_t)read(map_" + prefix.lower() + "_" + cmd["cmd"].lower() + "_byte2[" + index + "]);\n")
          file.write("    }\n")
      elif( get_eeprom_size(cmd) == 4 ):
          file.write("    if (get_eeprom_status() == EEPROM_STATUS_PRESENT)\n    {\n");
          file.write("        load_" + prefix.lower() + "_" + cmd["cmd"].lower() + "_val = (uint32_t)read(map_" + prefix.lower() + "_" + cmd["cmd"].lower() + "_byte1[" + index + "]);\n")
          file.write("        load_" + prefix.lower() + "_" + cmd["cmd"].lower() + "_val = ((uint32_t)load_" + prefix.lower() + "_" + cmd["cmd"].lower() + "_val << 8) | " + "(uint32_t)read(map_" + prefix.lower() + "_" + cmd["cmd"].lower() + "_byte2[" + index + "]);\n")
          file.write("        load_" + prefix.lower() + "_" + cmd["cmd"].lower() + "_val = ((uint32_t)load_" + prefix.lower() + "_" + cmd["cmd"].lower() + "_val << 16) | " + "(uint32_t)read(map_" + prefix.lower() + "_" + cmd["cmd"].lower() + "_byte3[" + index + "]);\n")
          file.write("        load_" + prefix.lower() + "_" + cmd["cmd"].lower() + "_val = ((uint32_t)load_" + prefix.lower() + "_" + cmd["cmd"].lower() + "_val << 24) | " + "(uint32_t)read(map_" + prefix.lower() + "_" + cmd["cmd"].lower() + "_byte4[" + index + "]);\n")
          file.write("    }\n")
    else:
      if( get_eeprom_size(cmd) == 1 ):
          file.write("    if (get_eeprom_status() == EEPROM_STATUS_PRESENT)\n    {\n");
          file.write("        load_" + prefix.lower() + "_" + cmd["cmd"].lower() + "_val = (uint8_t)read(EEPROM_" + prefix.upper() + "_" + cmd["cmd"].upper() + "_BYTE1);\n")
          file.write("    }\n")
      elif( get_eeprom_size(cmd) == 2 ):
          file.write("    if (get_eeprom_status() == EEPROM_STATUS_PRESENT)\n    {\n");
          file.write("        load_" + prefix.lower() + "_" + cmd["cmd"].lower() + "_val = (uint16_t)read(EEPROM_" + prefix.upper() + "_" + cmd["cmd"].upper() + "1);\n")
          file.write("        load_" + prefix.lower() + "_" + cmd["cmd"].lower() + "_val = ((uint16_t)load_" + prefix.lower() + "_" + cmd["cmd"].lower() + "_val << 8) | " + "(uint16_t)read(EEPROM_" + prefix.upper() + "_" + cmd["cmd"].upper() + "_BYTE2);\n")
          file.write("    }\n")
      elif( get_eeprom_size(cmd) == 4 ):
          file.write("    if (get_eeprom_status() == EEPROM_STATUS_PRESENT)\n    {\n");
          file.write("        load_" + prefix.lower() + "_" + cmd["cmd"].lower() + "_val = (uint32_t)read(EEPROM_" + prefix.lower() + "_" + cmd["cmd"].lower() + "1);\n")
          file.write("        load_" + prefix.lower() + "_" + cmd["cmd"].lower() + "_val = ((uint32_t)load_" + prefix.lower() + "_" + cmd["cmd"].lower() + "_val << 8) | " + "(uint32_t)read(EEPROM_" + prefix.upper() + "_" + cmd["cmd"].upper() + "_BYTE2);\n")
          file.write("        load_" + prefix.lower() + "_" + cmd["cmd"].lower() + "_val = ((uint32_t)load_" + prefix.lower() + "_" + cmd["cmd"].lower() + "_val << 16) | " + "(uint32_t)read(EEPROM_" + prefix.upper() + "_" + cmd["cmd"].upper() + "_BYTE3);\n")
          file.write("        load_" + prefix.lower() + "_" + cmd["cmd"].lower() + "_val = ((uint32_t)load_" + prefix.lower() + "_" + cmd["cmd"].lower() + "_val << 24) | " + "(uint32_t)read(EEPROM_" + prefix.upper() + "_" + cmd["cmd"].upper() + "_BYTE4);\n")
          file.write("    }\n")
         

    file.write("    return load_" + prefix.lower() + "_" + cmd["cmd"].lower() + "_val;\n")
   file.write( "}\n\n" )

def process_struct( prefix, cmd, depth ):
   write_comment_block( config_h, prefix, cmd, depth )
   write_custom_struct( config_h, prefix, cmd, depth )
   write_verify_declare( config_h, prefix, cmd, depth )
   write_load_source( config_c, prefix, cmd, depth )
   write_verify_source(config_c, prefix, cmd, depth )
   write_get_declare( config_h, prefix, cmd, depth )
   write_set_declare( config_h, prefix, cmd, depth )

for i, struct in enumerate(config["config"]["struct_list"]):
    if isinstance(struct, list):  # Check if the item is a list
        parent_struct = config["config"]["struct_list"][i - 1]  # Get the parent struct name (previous item)
        
        for sub_struct in struct:  # Iterate through the sublist
            print(f"Parent struct: {parent_struct}, Sub-struct: {sub_struct}")
            for cmd in config[sub_struct]:
                write_default_define(config_c, str(parent_struct) + "_" + sub_struct, cmd, 2)
    else:
        # If it's not a list, just process the struct
        for cmd in config[struct]:
            write_default_define(config_c, struct, cmd, 1)

config_c.write("\n")

for i, struct in enumerate(config["config"]["struct_list"]):
    if isinstance(struct, list):  # Check if the item is a list
        parent_struct = config["config"]["struct_list"][i - 1]  # Get the parent struct name (previous item)
        
        for sub_struct in struct:  # Iterate through the sublist
            print(f"Parent struct: {parent_struct}, Sub-struct: {sub_struct}")
            for cmd in config[sub_struct]:
                write_memory_organization( config_c, str(parent_struct) + "_" + sub_struct, cmd, 2)
    else:
        # If it's not a list, just process the struct
        for cmd in config[struct]:
            write_memory_organization( config_c, struct, cmd, 1)

config_c.write("\n")

for i, struct in enumerate(config["config"]["struct_list"]):
    if isinstance(struct, list):  # Check if the item is a list
        parent_struct = config["config"]["struct_list"][i - 1]  # Get the parent struct name (previous item)
        
        for sub_struct in struct:  # Iterate through the sublist
            print(f"Parent struct: {parent_struct}, Sub-struct: {sub_struct}")
            for cmd in config[sub_struct]:
                write_variables( config_c, str(parent_struct) + "_" + sub_struct, cmd, 2)
    else:
        # If it's not a list, just process the struct
        for cmd in config[struct]:
            write_variables( config_c, struct, cmd, 1)

config_c.write("\n\n")

for i, struct in enumerate(config["config"]["struct_list"]):
    if isinstance(struct, list):  # Check if the item is a list
        parent_struct = config["config"]["struct_list"][i - 1]  # Get the parent struct name (previous item)
        
        for sub_struct in struct:  # Iterate through the sublist
            print(f"Parent struct: {parent_struct}, Sub-struct: {sub_struct}")
            for cmd in config[sub_struct]:
              process_struct(str(parent_struct) + "_" + sub_struct, cmd, 2)
    else:
      for cmd in config[struct]:
        process_struct( struct, cmd, 1 )

config_h.write("\n#ifdef __cplusplus\n")
config_h.write("}\n")
config_h.write("#endif\n\n")
config_h.write("#endif /* KE_CONFIG_H */")

config_c.close()
config_h.close()

print("Usage: " + str(TotalByteCount) + "B of 8000B (64kbit) (" + str(TotalByteCount/80) + "%)")
print("Load time: " + str(TotalByteCount * 0.020) + "ms")