import { useQueryClient } from "@tanstack/react-query";
import { useEffect } from "react";

import { queryKeys } from "@/api/queries";
import { decodeEvent, PORTAL_EVENT_NAMES } from "@/api/sse";
import { type ConnectionState, usePortalStore } from "@/stores/portalStore";

const RETRY_MIN_MS = 1_000;
const RETRY_MAX_MS = 10_000;

/**
 * Subscribes to the pet server's `/api/events` SSE stream for the lifetime of the component.
 *
 * The browser reconnects by itself after network drops (and resends `Last-Event-ID`). When the
 * server is absent the dev proxy answers with an HTTP error, which makes EventSource give up, so
 * this hook re-creates the connection with a capped backoff. `state`/`status` events go to the
 * query cache; everything else goes to the portal store.
 */
export function useEventStream(url = "/api/events"): ConnectionState {
  const queryClient = useQueryClient();
  const ingest = usePortalStore((store) => store.ingest);
  const setConnection = usePortalStore((store) => store.setConnection);
  const connection = usePortalStore((store) => store.connection);

  useEffect(() => {
    let source: EventSource | null = null;
    let retryTimer: ReturnType<typeof setTimeout> | null = null;
    let retryMs = RETRY_MIN_MS;
    let disposed = false;

    const handleFrame = (name: (typeof PORTAL_EVENT_NAMES)[number]) => (message: Event) => {
      const { data, lastEventId } = message as MessageEvent<string>;
      const event = decodeEvent(name, data, lastEventId);
      if (event === null) return;
      if (event.type === "state") queryClient.setQueryData(queryKeys.state, event.state);
      else if (event.type === "status") queryClient.setQueryData(queryKeys.status, event.status);
      else ingest(event);
    };

    const connect = () => {
      if (disposed) return;
      setConnection("connecting");
      const next = new EventSource(url);
      source = next;
      for (const name of PORTAL_EVENT_NAMES) next.addEventListener(name, handleFrame(name));
      next.onopen = () => {
        retryMs = RETRY_MIN_MS;
        setConnection("open");
        queryClient.invalidateQueries({ queryKey: queryKeys.state });
        queryClient.invalidateQueries({ queryKey: queryKeys.status });
      };
      next.onerror = () => {
        if (next.readyState === EventSource.CLOSED) {
          setConnection("closed");
          next.close();
          retryTimer = setTimeout(connect, retryMs);
          retryMs = Math.min(retryMs * 2, RETRY_MAX_MS);
        } else {
          setConnection("connecting");
        }
      };
    };

    connect();
    return () => {
      disposed = true;
      if (retryTimer !== null) clearTimeout(retryTimer);
      source?.close();
      setConnection("closed");
    };
  }, [url, queryClient, ingest, setConnection]);

  return connection;
}
