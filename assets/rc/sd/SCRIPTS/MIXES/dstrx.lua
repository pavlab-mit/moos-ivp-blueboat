---------------------------------------------------------------
-- dstrx.lua -- decode vehicle custom telemetry -> "Dst" sensor
--
-- The vehicle computes distance from its own MOOS origin
-- (hypot(NAV_X, NAV_Y)) and sends it in a CRSF custom-telemetry
-- frame (type 0x80, subtype 0xB1, uint16 big-endian meters).
-- EdgeTX does not decode 0x80 natively; this script pops those
-- frames and publishes the value as a real telemetry sensor
-- named "Dst" (meters), usable in widgets, logical switches and
-- alarms like any discovered sensor.
--
-- No per-vehicle configuration: the origin lives on the vehicle,
-- in the mission file, where it belongs. Identical script on
-- every handset, works for boats and planes alike.
--
-- Install: SD /SCRIPTS/MIXES/dstrx.lua, then Model menu ->
-- Custom scripts -> add "dstrx".
---------------------------------------------------------------

local SENSOR_ID = 0x5FD0
local FRAME_CUSTOM = 0x80
local SUB_DISTANCE = 0xB1

local function init()
end

local function run()
  -- Drain the queue each cycle; keep only the newest distance.
  local dist = nil
  local cmd, data = crossfireTelemetryPop()
  while cmd ~= nil do
    if cmd == FRAME_CUSTOM and data ~= nil and #data >= 3
       and data[1] == SUB_DISTANCE then
      dist = data[2] * 256 + data[3]
    end
    cmd, data = crossfireTelemetryPop()
  end
  if dist ~= nil then
    -- unit 9 = meters, precision 0
    setTelemetryValue(SENSOR_ID, 0, 0, dist, 9, 0, "Dst")
  end
end

return { init = init, run = run }
