export default function statusParser(
  rawString: string,
  numberOfKeys: number
): any {
  const commandList = rawString.split("\n").filter((command) => {
    try {
      return Object.keys(JSON.parse(command)).length === numberOfKeys;
    } catch (e) {
      return false;
    }
  });
  return commandList.length > 0
    ? JSON.parse(commandList[commandList.length - 1])
    : null;
}
