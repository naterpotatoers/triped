export interface LegPositions {
  [key: string]: {
    x: number;
    y: number;
    z: number;
  };
}

export const MOCK_LEG_POSITIONS: LegPositions = {
  frontLeft: {
    x: 0,
    y: 0,
    z: 0,
  },
  frontRight: {
    x: 0,
    y: 0,
    z: 0,
  },
  backLeft: {
    x: 0,
    y: 0,
    z: 0,
  },
  backRight: {
    x: 0,
    y: 0,
    z: 0,
  },
};
