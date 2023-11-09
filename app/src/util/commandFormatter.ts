import { LegPositions } from "../types/LegPositions";

export function legPositionsCommandFormatter(
  legPositions: LegPositions
): string {
  // format into json string
  const command = JSON.stringify(legPositions);
  return command;
}
