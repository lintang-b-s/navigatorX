import http from "k6/http";
import { sleep, check } from "k6";
export const options = {
  stages: [
    { duration: "1m", target: 1200 }, // ramp up
    { duration: "1m", target: 1200 }, // peak
  ],
};

function getRandomInRange(from, to, fixed) {
  return (Math.random() * (to - from) + from).toFixed(fixed) * 1;
}

const route = [
  [-7.79961, 110.36307, -7.77495, 110.38496],
  [-7.79961, 110.36307, -7.56004, 110.77931],
  [-7.5548, 110.74133, -7.54902, 110.78047],
  [-7.58199, 110.83137, -7.5548, 110.80914],
  [-7.70904, 110.59705, -7.5654, 110.83424],
];

export default () => {
  // if you get 400, its because source and destination not connected
  const srcLat = getRandomInRange(-7.84792742160175, -7.717121610226075, 5);
  const srcLon = getRandomInRange(110.29874470172537, 110.44762570363511, 5);

  const dstLat = getRandomInRange(-7.84792742160175, -7.717121610226075, 5);
  const dstLon = getRandomInRange(110.29874470172537, 110.44762570363511, 5);
  const reqBody = {
    src_lat: srcLat,
    src_lon: srcLon,
    dst_lat: dstLat,
    dst_lon: dstLon,
  };

  const res = http.get(
    `http://localhost:5000/api/navigations/shortest-path?src_lat=${reqBody.src_lat}&src_lon=${reqBody.src_lon}&dst_lat=${reqBody.dst_lat}&dst_lon=${reqBody.dst_lon}`,
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
