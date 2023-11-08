import { useEffect, useRef, useState } from "react";
import statusParser from "../util/statusParser";

export default function SerialWifi({
  setStatus,
  numberOfKeys,
}: {
  setStatus: React.Dispatch<React.SetStateAction<any>>;
  numberOfKeys: number;
}) {
  let rawSerial: string = "";
  const port = useRef<SerialPort | undefined>(undefined);
  const reader = useRef<ReadableStreamDefaultReader>();
  const writer = useRef<WritableStreamDefaultWriter>();
  const [isConnected, setIsConnected] = useState(false);
  const [isDtrModeEnabled, setIsDtrModeEnabled] = useState(false);
  const [serverAddress, setServerAddress] = useState(
    "http://localhost:5000/arm"
  );

  async function connect() {
    try {
      port.current = await navigator.serial.requestPort();
      await port.current.open({ baudRate: 38400 });
      await port.current.setSignals({
        dataTerminalReady: false,
        requestToSend: false,
      });
      reader.current = port.current?.readable?.getReader();
      writer.current = port.current?.writable?.getWriter();
      setIsConnected(true);
    } catch (error) {
      console.error(error);
      setIsConnected(false);
    }
  }

  function disconnect() {
    try {
      if (reader.current) {
        reader.current.cancel();
        writer.current?.abort();
        reader.current.releaseLock();
        writer.current?.releaseLock();
        port.current?.close();
        port.current = undefined;
        reader.current = undefined;
        writer.current = undefined;
        setIsConnected(false);
        setIsDtrModeEnabled(false);
      }
    } catch (error) {
      console.error(error);
      setIsConnected(true);
    }
  }

  async function handleReadWrite() {
    while (isConnected && reader.current) {
      const { value } = await reader.current.read();
      let decoded = await new TextDecoder().decode(value);
      rawSerial += await decoded;
      const command = statusParser(rawSerial, numberOfKeys);
      if (command !== null) {
        rawSerial = "";
        console.log(command);
        fetch(serverAddress, {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
          },
          body: JSON.stringify(command),
        })
          .then((response) => response.json())
          .then((data) => {
            console.log("Success:", data);
            setStatus(data);
          })
          .catch((error) => {
            console.error("Error:", error);
            setStatus({
              message: error.message,
              url: error.config.url,
            });
          });
      }
      try {
        // let response = await axios.get(serverAddress + "/status");
        let response = await fetch(serverAddress + "/status");
        setStatus(response);
      } catch (error) {
        disconnect();
        setStatus({
          message:
            "An error occurred while trying to fetch status from the server",
        });
      }
    }
  }

  async function toggleDataTerminalMode() {
    try {
      if (port.current) {
        await port.current.setSignals({
          dataTerminalReady: !isDtrModeEnabled,
          requestToSend: !isDtrModeEnabled,
        });
        setIsDtrModeEnabled(!isDtrModeEnabled);
      }
    } catch (error) {
      console.error(error);
    }
  }

  useEffect(() => {
    handleReadWrite();
  }, [isConnected]);

  return (
    <>
      <input
        type="text"
        value={serverAddress}
        onChange={(e) => setServerAddress(e.target.value)}
      />
      <div className="btn-group">
        {isDtrModeEnabled ? (
          <button
            className="btn btn__danger"
            onClick={() => toggleDataTerminalMode()}
          >
            Toggle DTR OFF
          </button>
        ) : (
          <button
            className="btn btn__primary"
            onClick={() => toggleDataTerminalMode()}
          >
            Toggle DTR ON
          </button>
        )}
        {isConnected ? (
          <button className="btn btn__danger" onClick={() => disconnect()}>
            Disconnect
          </button>
        ) : (
          <button className="btn btn__primary" onClick={() => connect()}>
            Connect
          </button>
        )}
      </div>
    </>
  );
}
