local input = require('openmw.input')
local util = require('openmw.util')
local async = require('openmw.async')
local storage = require('openmw.storage')
local types = require('openmw.types')
local self = require('openmw.self')

local NPC = types.NPC

local moveActions = {
    'MoveForward',
    'MoveBackward',
    'MoveLeft',
    'MoveRight'
}
for _, key in ipairs(moveActions) do
    local smoothKey = 'Smooth' .. key
    input.registerAction {
        key = smoothKey,
        l10n = 'OMWControls',
        name = smoothKey .. '_name',
        description = smoothKey .. '_description',
        type = input.ACTION_TYPE.Range,
        defaultValue = 0,
    }
end

local settings = storage.playerSection('SettingsOMWControls')

local function shouldAlwaysRun(actor)
    return actor.controls.sneak or not NPC.isOnGround(actor) or NPC.isSwimming(actor)
end

local function remapToWalkRun(actor, inputMovement)
    if shouldAlwaysRun(actor) then
        return true, inputMovement
    end
    local normalizedInput, inputSpeed = inputMovement:normalize()
    local switchPoint = 0.5
    if inputSpeed < switchPoint then
        return false, inputMovement * 2
    else
        local matchWalkingSpeed = NPC.getWalkSpeed(actor) / NPC.getRunSpeed(actor)
        local runSpeedRatio = 2 * (inputSpeed - switchPoint) * (1 - matchWalkingSpeed) + matchWalkingSpeed
        return true, normalizedInput * math.min(1, runSpeedRatio)
    end
end

local function computeSmoothMovement()
    local controllerInput = util.vector2(
        input.getAxisValue(input.CONTROLLER_AXIS.MoveForwardBackward),
        input.getAxisValue(input.CONTROLLER_AXIS.MoveLeftRight)
    )
    return remapToWalkRun(self, controllerInput)
end

-- Every binding below needs the same per-frame values; computing them in
-- each (four SmoothMove bindings, four Move bindings and Run) repeated the
-- axis reads, the ground/swim/sneak checks, the walk and run speeds and the
-- settings lookups up to five times a frame. Bindings run before onFrame,
-- so onFrame starts the next frame's cache.
local frame = 0
local cachedFrame = -1
local cachedSmoothEnabled, cachedRun, cachedMovement
local function perFrame()
    if cachedFrame ~= frame then
        cachedFrame = frame
        cachedSmoothEnabled = settings:get('smoothControllerMovement')
        cachedRun, cachedMovement = computeSmoothMovement()
    end
end

local function bindSmoothMove(key, axis, direction)
    local smoothKey = 'Smooth' .. key
    input.bindAction(smoothKey, async:callback(function()
        perFrame()
        return math.max(direction * cachedMovement[axis], 0)
    end), {})
    input.bindAction(key, async:callback(function(_, standardMovement, smoothMovement)
        perFrame()
        if not cachedSmoothEnabled then
            return standardMovement
        end

        if smoothMovement > 0 then
            return smoothMovement
        else
            return standardMovement
        end
    end), { smoothKey })
end

bindSmoothMove('MoveForward', 'x', -1)
bindSmoothMove('MoveBackward', 'x', 1)
bindSmoothMove('MoveRight', 'y', 1)
bindSmoothMove('MoveLeft', 'y', -1)

input.bindAction('Run', async:callback(function(_, run)
    perFrame()
    if not cachedSmoothEnabled then
        return run
    end
    if cachedMovement:length2() > 0 then
        -- ignore always run
        return cachedRun ~= settings:get('alwaysRun')
    else
        return run
    end
end), {})

return {
    engineHandlers = {
        onFrame = function()
            frame = frame + 1
        end,
    },
}
