import http from "k6/http";
import { sleep, check } from "k6";
export const options = {
  stages: [
    { duration: "1m", target: 700 }, // ramp up
    { duration: "1m", target: 700 }, // peak
  ],
};

export default () => {
  const reqBody = {
    src_lat: -7.550263588614922, // surakarta
    src_lon: 110.78206617571915,
    dst_lat: -8.024167150460844, // yogyakarta (parangtritis)
    dst_lon: 110.32986653162467,
  };

  const res = http.post(
    "http://localhost:5000/api/navigations/shortest-path",
    JSON.stringify(reqBody),
    {
      headers: {
        "Content-Type": "application/json",
        Accept: "application/json",
      },
    }
  );
  check(res, { 200: (r) => r.status === 200 });
  sleep(1);
};
