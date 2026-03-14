-- create protocol
local lpg_proto = Proto("lpg", "LPG Telemetry Protocol")

-- define fields
local f_map           = ProtoField.float ("lpg.map",           "MAP",              base.NONE, nil, nil, "Manifold Absolute Pressure (kPa)")
local f_lpg_bar       = ProtoField.float ("lpg.lpg_bar",       "LPG Pressure",     base.NONE, nil, nil, "LPG tank pressure (bar)")
local f_lpg_temp      = ProtoField.uint16("lpg.lpg_temp",      "LPG Temp",         base.DEC,  nil, nil, "LPG temperature (°C)")
local f_red_temp      = ProtoField.uint16("lpg.red_temp",      "Reducer Temp",     base.DEC,  nil, nil, "Reducer/vaporiser temperature (°C)")
local f_internal_temp = ProtoField.uint8 ("lpg.internal_temp", "Internal Temp",    base.DEC,  nil, nil, "ECU internal temperature (°C)")
local f_rpm           = ProtoField.uint16("lpg.rpm",           "RPM",              base.DEC,  nil, nil, "Engine speed (rev/min)")
local f_battery       = ProtoField.float ("lpg.battery_v",     "Battery Voltage",  base.NONE, nil, nil, "Supply voltage (V)")
local f_load          = ProtoField.uint16("lpg.load",          "Load",             base.DEC,  nil, nil, "Engine load (%)")
local f_pcm_real      = ProtoField.float ("lpg.pcm_real",      "PCM Pressure Real",base.NONE, nil, nil, "PCM real pressure (kPa)")
local f_pcm           = ProtoField.float ("lpg.pcm",           "PCM Pressure",     base.NONE, nil, nil, "PCM pressure (kPa)")
local f_pb1           = ProtoField.float ("lpg.pb_1",          "PB #1",            base.NONE, nil, nil, "Petrol injector #1 pulse-width pressure (kPa)")
local f_pb2           = ProtoField.float ("lpg.pb_2",          "PB #2",            base.NONE, nil, nil, "Petrol injector #2 pulse-width pressure (kPa)")
local f_pb3           = ProtoField.float ("lpg.pb_3",          "PB #3",            base.NONE, nil, nil, "Petrol injector #3 pulse-width pressure (kPa)")
local f_pb4           = ProtoField.float ("lpg.pb_4",          "PB #4",            base.NONE, nil, nil, "Petrol injector #4 pulse-width pressure (kPa)")
local f_lpg1          = ProtoField.float ("lpg.lpg_1",         "LPG Injector #1",  base.NONE, nil, nil, "LPG injector #1 pulse-width (kPa)")
local f_lpg2          = ProtoField.float ("lpg.lpg_2",         "LPG Injector #2",  base.NONE, nil, nil, "LPG injector #2 pulse-width (kPa)")
local f_lpg3          = ProtoField.float ("lpg.lpg_3",         "LPG Injector #3",  base.NONE, nil, nil, "LPG injector #3 pulse-width (kPa)")
local f_lpg4          = ProtoField.float ("lpg.lpg_4",         "LPG Injector #4",  base.NONE, nil, nil, "LPG injector #4 pulse-width (kPa)")
local f_stft          = ProtoField.uint16 ("lpg.stft",          "Short-term fuel trim",  base.DEC, nil, nil, "Short-term fuel trim (%)")


-- register fields
lpg_proto.fields = {
    f_map, f_lpg_bar, f_lpg_temp, f_red_temp,
    f_internal_temp, f_rpm, f_battery, f_load,
    f_pcm_real, f_pcm,
    f_pb1, f_pb2, f_pb3, f_pb4,
    f_lpg1, f_lpg2, f_lpg3, f_lpg4,
    f_stft, 
}

-- expert-info field for display anomalies
local ef_truncated = ProtoExpert.new(
    "lpg.truncated", "Packet too short for LPG telemetry",
    expert.group.MALFORMED, expert.severity.ERROR
)
lpg_proto.experts = { ef_truncated }

-- ── helpers ───────────────────────────────────────────────────────────────────

local function add_scaled(subtree, field, tvb_range, raw, scale, fmt, unit)
    local scaled = raw * scale
    local item = subtree:add(field, tvb_range, scaled)
    item:set_text(string.format("%s: %s %s",
        field.name, string.format(fmt, scaled), unit))
    return item
end

-- ── dissector ─────────────────────────────────────────────────────────────────

function lpg_proto.dissector(buffer, pinfo, tree)

    local buf_len = buffer:len()

    -- Furthest byte touched: load at 0x54–0x55
    local MIN_LEN = 0x56
    if buf_len < MIN_LEN then
        local subtree = tree:add(lpg_proto, buffer(), "LPG Telemetry [TRUNCATED]")
        subtree:add_proto_expert_info(ef_truncated,
            string.format("Need ≥ %d bytes, got %d", MIN_LEN, buf_len))
        return
    end

    pinfo.cols.protocol = "LPG"

    local subtree = tree:add(lpg_proto, buffer(), "LPG Telemetry")

    -- ── PCM Real  0x04–0x05  big-endian uint16 × 0.005 → kPa ────────────
    add_scaled(subtree, f_pcm_real,
        buffer(0x04, 2), buffer(0x04, 2):uint(), 0.005, "%.3f", "kPa")

    -- ── PCM  0x08–0x09  big-endian uint16 × 0.005 → kPa ─────────────────
    add_scaled(subtree, f_pcm,
        buffer(0x08, 2), buffer(0x08, 2):uint(), 0.005, "%.3f", "kPa")

    -- ── MAP  0x0C–0x0D  big-endian uint16 × 0.01 → kPa ──────────────────
    add_scaled(subtree, f_map,
        buffer(0x0C, 2), buffer(0x0C, 2):uint(), 0.01, "%.2f", "kPa")

    -- ── LPG Pressure  0x0F–0x10  little-endian uint16 × 0.01 → bar ───────
    add_scaled(subtree, f_lpg_bar,
        buffer(0x0F, 2), buffer(0x0F, 2):le_uint(), 0.01, "%.2f", "bar")

    -- ── LPG Temp  0x11–0x12  little-endian uint16 → °C ───────────────────
    local lpg_temp_val = buffer(0x11, 2):le_uint()
    subtree:add(f_lpg_temp, buffer(0x11, 2), lpg_temp_val)
        :set_text(string.format("LPG Temp: %d °C", lpg_temp_val))

    -- ── Reducer Temp  0x13–0x14  little-endian uint16 → °C ───────────────
    local red_temp_val = buffer(0x13, 2):le_uint()
    subtree:add(f_red_temp, buffer(0x13, 2), red_temp_val)
        :set_text(string.format("Reducer Temp: %d °C", red_temp_val))

    -- ── Internal Temp  0x15  uint8 → °C ───────────────────────────────────
    local int_temp_val = buffer(0x15, 1):uint()
    subtree:add(f_internal_temp, buffer(0x15, 1), int_temp_val)
        :set_text(string.format("Internal Temp: %d °C", int_temp_val))

    -- ── RPM  0x16–0x17  big-endian uint16 → rpm ───────────────────────────
    local rpm_val = buffer(0x16, 2):uint()
    subtree:add(f_rpm, buffer(0x16, 2), rpm_val)
        :set_text(string.format("RPM: %d rpm", rpm_val))

    -- ── Battery Voltage  0x18–0x19  big-endian uint16 × 0.001 → V ─────────
    add_scaled(subtree, f_battery,
        buffer(0x18, 2), buffer(0x18, 2):uint(), 0.001, "%.3f", "V")

    -- ── PB #1  0x32–0x33  big-endian uint16 × 0.005 → kPa ────────────────
    add_scaled(subtree, f_pb1,
        buffer(0x32, 2), buffer(0x32, 2):uint(), 0.005, "%.3f", "kPa")

    -- ── PB #2  0x36–0x37  big-endian uint16 × 0.005 → kPa ────────────────
    add_scaled(subtree, f_pb2,
        buffer(0x36, 2), buffer(0x36, 2):uint(), 0.005, "%.3f", "kPa")

    -- ── PB #3  0x3A–0x3B  big-endian uint16 × 0.005 → kPa ────────────────
    add_scaled(subtree, f_pb3,
        buffer(0x3A, 2), buffer(0x3A, 2):uint(), 0.005, "%.3f", "kPa")

    -- ── PB #4  0x3E–0x3F  big-endian uint16 × 0.005 → kPa ────────────────
    add_scaled(subtree, f_pb4,
        buffer(0x3E, 2), buffer(0x3E, 2):uint(), 0.005, "%.3f", "kPa")

    -- ── LPG Injector #1  0x42–0x43  big-endian uint16 × 0.005 → kPa ──────
    add_scaled(subtree, f_lpg1,
        buffer(0x42, 2), buffer(0x42, 2):uint(), 0.005, "%.3f", "kPa")

    -- ── LPG Injector #2  0x46–0x47  big-endian uint16 × 0.005 → kPa ──────
    add_scaled(subtree, f_lpg2,
        buffer(0x46, 2), buffer(0x46, 2):uint(), 0.005, "%.3f", "kPa")

    -- ── LPG Injector #3  0x4A–0x4B  big-endian uint16 × 0.005 → kPa ──────
    add_scaled(subtree, f_lpg3,
        buffer(0x4A, 2), buffer(0x4A, 2):uint(), 0.005, "%.3f", "kPa")

    -- ── LPG Injector #4  0x4E–0x4F  big-endian uint16 × 0.005 → kPa ──────
    add_scaled(subtree, f_lpg4,
        buffer(0x4E, 2), buffer(0x4E, 2):uint(), 0.005, "%.3f", "kPa")

    -- ── PCM  0x08–0x09  big-endian uint16 × 0.005 → kPa ─────────────────
    add_scaled(subtree, f_stft,
        buffer(0x5C, 2), buffer(0x5C, 2):int(), 1, "%.3f", "%")

    -- ── Load  0x54–0x55  little-endian uint16 → % ─────────────────────────
    local load_val = buffer(0x54, 2):le_uint()
    subtree:add(f_load, buffer(0x54, 2), load_val)
        :set_text(string.format("Load: %d %%", load_val))
end

-- ── register on UDP 5555 ──────────────────────────────────────────────────────
DissectorTable.get("udp.port"):add(5555, lpg_proto)