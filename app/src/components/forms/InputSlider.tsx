import { useState, useEffect } from "react";

export default function InputSlider({
  name,
  value,
  onChange,
  min = 0,
  max = 100,
  label = "",
  disabled = false,
  autocomplete = "off",
}: {
  name: string;
  value: number;
  onChange: (e: React.ChangeEvent<HTMLInputElement>) => void;
  min?: number;
  max?: number;
  label?: string;
  disabled?: boolean;
  autocomplete?: string;
}) {
  const [inputText, setInputText] = useState(value);
  const handleTextInputChange = (e) => {
    if (e.target.value > max) {
      e.target.value = max;
    } else if (e.target.value < min) {
      e.target.value = min;
    }
    setInputText(e.target.value);
  };

  useEffect(() => {
    setInputText(value);
  }, [value]);

  return (
    <>
      <label className="label_lg"> {label}</label>
      <div className="slider-group">
        <input
          className="input-text btn slider-input"
          type="number"
          name={name}
          max={max}
          min={min}
          value={inputText}
          disabled={disabled}
          onBlur={onChange}
          onChange={handleTextInputChange}
          autoComplete={autocomplete}
        />
        <input
          className="slider"
          type="range"
          name={name}
          max={max}
          min={min}
          value={value}
          disabled={disabled}
          onChange={onChange}
        />
      </div>
    </>
  );
}
