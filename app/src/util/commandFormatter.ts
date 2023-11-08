import { LegPositions } from "../types/LegPositions";

export function legPositionsCommandFormatter(
  legPositions: LegPositions
): string {
  const legPositionsArray = Object.keys(legPositions).map((leg) => {
    return `${leg} ${legPositions[leg].x} ${legPositions[leg].y} ${legPositions[leg].z}`;
  });
  return legPositionsArray.join("\n");
}
