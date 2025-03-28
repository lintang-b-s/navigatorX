import http from "k6/http";
import { sleep, check } from "k6";
export const options = {
  stages: [
    { duration: "1m", target: 1200 }, // ramp up
    { duration: "1m", target: 1200 }, // peak
  ],
};

const route = [[-7.79961, 110.36307, -7.77495, 110.38496], [-7.79961, 110.36307,-7.56004, 110.77931],
        [-7.55480, 110.74133, -7.54902, 110.78047], [-7.58199, 110.83137, -7.55480, 110.80914],
      [-7.70904, 110.59705, -7.56540, 110.83424]]

export default () => {
  
  const randomRoute = route[Math.floor(Math.random() * route.length)];

  const reqBody = {
    src_lat: randomRoute[0],
    src_lon: randomRoute[1],
    dst_lat: randomRoute[2],
    dst_lon:  randomRoute[3],
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
