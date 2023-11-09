import { useRef, useState } from "react";
import Wifi from "./components/Wifi";
import LegPositionsForm from "./components/LegPositionsForm";

function App() {
  const commands = useRef<string>("");
  const [status, setStatus] = useState();

  return (
    <div>
      <Wifi commands={commands} setStatus={setStatus} />
      <LegPositionsForm commands={commands} />
      <pre>
        <code>{JSON.stringify(status, null, 2)}</code>
      </pre>
    </div>
  );
}

export default App;
