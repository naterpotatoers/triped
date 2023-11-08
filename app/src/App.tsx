import { useRef, useState } from "react";
import Wifi from "./components/Wifi";
import LegPositionsForm from "./components/LegPositionsForm";

function App() {
  const commands = useRef<string>("");
  const [status, setStatus] = useState();
  const [telemetry, setTelemetry] = useState("wifi");

  return (
    <div>
      <Wifi commands={commands} setStatus={setStatus} />
      <LegPositionsForm commands={commands} />
    </div>
  );
}

export default App;
