import axios, { AxiosResponse } from "axios";

export const axiosInstance = axios.create({
  baseURL: "http://localhost:3000",
  // 다른 설정들...
});

interface Params {
  [key: string]: string | number;
}

const fetcher = async <T>(url: string, params?: Params): Promise<T> => {
  try {
    const response: AxiosResponse<T> = await axiosInstance.get(url, { params });
    return response.data;
  } catch (error) {
    throw error;
  }
};

export default fetcher;
