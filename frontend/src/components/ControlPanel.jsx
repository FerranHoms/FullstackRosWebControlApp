import React, { useState } from 'react';
import { controlCoffee } from '../api';

const CoffeeDispenser = () => {
  const [dispenserNumber, setDispenserNumber] = useState(1);
  const [durationSeconds, setDurationSeconds] = useState(0);
  const [brewAction, setBrewAction] = useState('timed');

  const handleSubmit = async (e) => {
    e.preventDefault();
    const data = {
      dispenser_number: dispenserNumber,
      duration_seconds: brewAction === 'timed' ? durationSeconds : 0,
      start: brewAction === 'start',
      stop: brewAction === 'stop',
    };
    try {
      await controlCoffee(data);
      alert('Coffee brew command sent successfully');
    } catch (error) {
      alert('Failed to send coffee brew command');
    }
  };

  return (
    <div className="flex flex-col items-center justify-center min-h-screen w-full bg-gradient-to-br from-yellow-100 to-orange-200 p-4">
      <h1 className="text-4xl font-bold mb-8 text-orange-700">☕ Coffee Dispenser ☕</h1>
      <form onSubmit={handleSubmit} className="w-full max-w-md bg-white shadow-md rounded px-8 pt-6 pb-8">
        <h2 className="text-2xl font-semibold mb-4 text-gray-800">Brew Your Coffee</h2>
        <div className="mb-4">
          <label className="block text-gray-700">Dispenser Number (1-8):</label>
          <input
            type="number"
            min="1"
            max="8"
            value={dispenserNumber}
            onChange={(e) => setDispenserNumber(parseInt(e.target.value))}
            className="border border-gray-300 p-2 rounded w-full text-gray-800"
          />
        </div>
        <div className="mb-4">
          <label className="block text-gray-700">Brew Action:</label>
          <select
            value={brewAction}
            onChange={(e) => setBrewAction(e.target.value)}
            className="border border-gray-300 p-2 rounded w-full text-gray-800"
          >
            <option value="timed">Timed Brew</option>
            <option value="start">Start Brewing (Indefinite)</option>
            <option value="stop">Stop Brewing</option>
          </select>
        </div>
        {brewAction === 'timed' && (
          <div className="mb-4">
            <label className="block text-gray-700">Brew Duration (seconds):</label>
            <input
              type="number"
              min="0"
              value={durationSeconds}
              onChange={(e) => setDurationSeconds(parseInt(e.target.value))}
              className="border border-gray-300 p-2 rounded w-full text-gray-800"
            />
          </div>
        )}
        <button
          type="submit"
          className="bg-amber-600 hover:bg-amber-800 text-white font-bold py-2 px-4 rounded w-full"
        >
          Brew Coffee
        </button>
      </form>
    </div>
  );
};

export default CoffeeDispenser;
