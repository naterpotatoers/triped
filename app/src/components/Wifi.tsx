import { useEffect, useState } from "react";

export default function Wifi({
  commands,
  setStatus,
}: {
  commands: React.MutableRefObject<string>;
  setStatus: React.Dispatch<React.SetStateAction<any>>;
}) {
  const [isConnected, setIsConnected] = useState(false);
  const [serverAddress, setServerAddress] = useState(
    "http://localhost:5000/quadruped"
  );

  function connect() {
    setIsConnected(true);
  }

  function disconnect() {
    setIsConnected(false);
  }

  async function writeCommands() {
    if (isConnected) {
      try {
        const responseStatus = await fetch(serverAddress, {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
          },
          body: commands.current,
        });
        const response = await responseStatus.json();
        console.log(response);
        setStatus(response);
      } catch (error) {
        console.log(error);
        disconnect();
        setStatus("Unable to post commands, verify backend is running");
      }
    }
  }

  useEffect(() => {
    const writeInterval = setInterval(() => {
      if (isConnected) {
        writeCommands();
      }
    }, 200);
    return () => clearInterval(writeInterval);
  }, [isConnected]);

  return (
    <>
      <input
        autoComplete="off"
        className="input-text"
        type="text"
        value={serverAddress}
        onChange={(e) => setServerAddress(e.target.value)}
      />
      {isConnected ? (
        <button className="btn btn__danger" onClick={disconnect}>
          Disconnect
        </button>
      ) : (
        <button className="btn btn__primary" onClick={connect}>
          Connect
        </button>
      )}
    </>
  );
}
