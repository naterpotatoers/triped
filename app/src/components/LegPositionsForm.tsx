import { useState, useEffect, useRef } from "react";
import { useGamepads } from "react-gamepads";
import { LegPositions, MOCK_LEG_POSITIONS } from "../types/LegPositions";
import { legPositionsCommandFormatter } from "../util/commandFormatter";
import InputSlider from "./forms/InputSlider";

export default function LegPositionsForm({
  commands,
}: {
  commands: React.MutableRefObject<string>;
}) {
  const [gamepad, setGamepad] = useState<Gamepad>();
  const [legPositionsCommands, setLegPositionsCommands] =
    useState<LegPositions>(MOCK_LEG_POSITIONS);
  const commandsRef = useRef(MOCK_LEG_POSITIONS);
  const gamepadRef = useRef(gamepad);

  useGamepads((gamepads) => {
    if (gamepads[0]) {
      setGamepad(gamepads[0]);
    }
  });

  useEffect(() => {
    gamepadRef.current = gamepad;
  }, [gamepad]);

  function updateCommands(newCommands: LegPositions) {
    commandsRef.current = newCommands;
    commands.current = legPositionsCommandFormatter(newCommands);
    setLegPositionsCommands(newCommands);
  }

  function updateController() {
    const leftStickX = gamepad?.axes[0];
    const leftStickY = gamepad?.axes[1];
    const rightStickX = gamepad?.axes[2];
    const rightStickY = gamepad?.axes[3];
    if (gamepadRef.current) {
      const newCommands = {
        ...commandsRef.current,
        // TODO: create mappings for each leg
      };
      console.log(newCommands);
      updateCommands(newCommands);
    }
  }

  useEffect(() => {
    const interval = setInterval(() => {
      updateController();
    }, 50);
    return () => clearInterval(interval);
  }, []);

  return (
    <form>
      <InputSlider
        name="frontLeft"
        value={legPositionsCommands.frontLeft.x}
        onChange={(e) =>
          updateCommands({
            ...legPositionsCommands,
            frontLeft: { ...legPositionsCommands.frontLeft, x: e.target.value },
          })
        }
        label="Front Left X"
        min={-100}
        max={100}
      />
      <InputSlider
        name="frontLeft"
        value={legPositionsCommands.frontLeft.y}
        onChange={(e) =>
          updateCommands({
            ...legPositionsCommands,
            frontLeft: { ...legPositionsCommands.frontLeft, y: e.target.value },
          })
        }
        label="Front Left Y"
        min={-100}
        max={100}
      />
      <InputSlider
        name="frontLeft"
        value={legPositionsCommands.frontLeft.z}
        onChange={(e) =>
          updateCommands({
            ...legPositionsCommands,
            frontLeft: { ...legPositionsCommands.frontLeft, z: e.target.value },
          })
        }
        label="Front Left Z"
        min={-100}
        max={100}
      />
      <InputSlider
        name="frontRight"
        value={legPositionsCommands.frontRight.x}
        onChange={(e) =>
          updateCommands({
            ...legPositionsCommands,
            frontRight: {
              ...legPositionsCommands.frontRight,
              x: e.target.value,
            },
          })
        }
        label="Front Right X"
        min={-100}
        max={100}
      />
      <InputSlider
        name="frontRight"
        value={legPositionsCommands.frontRight.y}
        onChange={(e) =>
          updateCommands({
            ...legPositionsCommands,
            frontRight: {
              ...legPositionsCommands.frontRight,
              y: e.target.value,
            },
          })
        }
        label="Front Right Y"
        min={-100}
        max={100}
      />
      <InputSlider
        name="frontRight"
        value={legPositionsCommands.frontRight.z}
        onChange={(e) =>
          updateCommands({
            ...legPositionsCommands,
            frontRight: {
              ...legPositionsCommands.frontRight,
              z: e.target.value,
            },
          })
        }
        label="Front Right Z"
        min={-100}
        max={100}
      />
      <InputSlider
        name="backLeft"
        value={legPositionsCommands.backLeft.x}
        onChange={(e) =>
          updateCommands({
            ...legPositionsCommands,
            backLeft: { ...legPositionsCommands.backLeft, x: e.target.value },
          })
        }
        label="Back Left X"
        min={-100}
        max={100}
      />
      <InputSlider
        name="backLeft"
        value={legPositionsCommands.backLeft.y}
        onChange={(e) =>
          updateCommands({
            ...legPositionsCommands,
            backLeft: { ...legPositionsCommands.backLeft, y: e.target.value },
          })
        }
        label="Back Left Y"
        min={-100}
        max={100}
      />
      <InputSlider
        name="backLeft"
        value={legPositionsCommands.backLeft.z}
        onChange={(e) =>
          updateCommands({
            ...legPositionsCommands,
            backLeft: { ...legPositionsCommands.backLeft, z: e.target.value },
          })
        }
        label="Back Left Z"
        min={-100}
        max={100}
      />
      <InputSlider
        name="backRight"
        value={legPositionsCommands.backRight.x}
        onChange={(e) =>
          updateCommands({
            ...legPositionsCommands,
            backRight: {
              ...legPositionsCommands.backRight,
              x: e.target.value,
            },
          })
        }
        label="Back Right X"
        min={-100}
        max={100}
      />
      <InputSlider
        name="backRight"
        value={legPositionsCommands.backRight.y}
        onChange={(e) =>
          updateCommands({
            ...legPositionsCommands,
            backRight: {
              ...legPositionsCommands.backRight,
              y: e.target.value,
            },
          })
        }
        label="Back Right Y"
        min={-100}
        max={100}
      />
      <InputSlider
        name="backRight"
        value={legPositionsCommands.backRight.z}
        onChange={(e) =>
          updateCommands({
            ...legPositionsCommands,
            backRight: {
              ...legPositionsCommands.backRight,
              z: e.target.value,
            },
          })
        }
        label="Back Right Z"
        min={-100}
        max={100}
      />
    </form>
  );
}
