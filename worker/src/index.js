export default {
  async fetch(request, env) {
    const url = new URL(request.url);
    const origin = new Request(`${env.ORIGIN}${url.pathname}${url.search}`, request);
    try {
      return await env.DRONE.fetch(origin);
    } catch (e) {
      return new Response("The S5C Elite PC is offline - turn it on and the drone link comes back by itself.", { status: 503 });
    }
  },
};
