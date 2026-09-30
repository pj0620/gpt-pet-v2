import { useEffect, useState } from "react";

import { imageUrl } from "@/api/client";
import { usePortalStore } from "@/stores/portalStore";
import type { ImageName } from "@/types/api";

interface LiveImageProps {
  name: ImageName;
  alt: string;
  testId: string;
  placeholder: string;
}

/**
 * Shows the latest server-side image for `name`. The URL changes when the stream announces a new
 * version; the new frame is preloaded and swapped in only once it has loaded, so there is no flicker.
 */
export function LiveImage({ name, alt, testId, placeholder }: LiveImageProps) {
  const version = usePortalStore((store) => store.imageVersions[name]);
  const connection = usePortalStore((store) => store.connection);
  const [loadedSrc, setLoadedSrc] = useState<string | null>(null);

  useEffect(() => {
    if (connection !== "open") return;
    const url = imageUrl(name, version);
    let cancelled = false;
    const image = new Image();
    image.onload = () => {
      if (!cancelled) setLoadedSrc(url);
    };
    image.src = url;
    return () => {
      cancelled = true;
    };
  }, [name, version, connection]);

  if (loadedSrc === null) {
    return (
      <div
        className="flex h-full items-center justify-center p-3 text-xs text-muted-foreground"
        data-testid={`${testId}-placeholder`}
      >
        {placeholder}
      </div>
    );
  }
  return (
    <div className="flex h-full items-center justify-center bg-black/40 p-2">
      <img
        src={loadedSrc}
        alt={alt}
        data-testid={testId}
        className="max-h-full max-w-full object-contain"
        decoding="async"
      />
    </div>
  );
}
