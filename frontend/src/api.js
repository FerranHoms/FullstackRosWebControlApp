import axios from 'axios';

const api = axios.create({
  baseURL: 'http://localhost:5000',
});

export const controlSolenoid = (data) => api.post('/solenoid', data);
export const controlPump = (data) => api.post('/pump', data);