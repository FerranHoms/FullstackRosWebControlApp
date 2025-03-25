import axios from 'axios';

const api = axios.create({
  baseURL: '/',
});

export const controlCoffee = (data) => api.post('/api/coffee', data);