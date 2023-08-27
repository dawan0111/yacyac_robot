import { axiosInstance } from "./fetcher";

interface History {
  pill1: number;
  pill2: number;
  pill3: number;
  pill4: number;
  pill5: number;
  pill6: number;
  pill7: number;
  pill8: number;
  userId: number;
  accept: boolean;
}

export const saveHistory = async (history: History): Promise<History> => {
  const { data } = await axiosInstance.post<History>(`/historys`, history);
  return data;
};
