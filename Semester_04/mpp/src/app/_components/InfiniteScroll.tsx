"use client";

import React, { useEffect, useRef, useState } from "react";

interface InfiniteScrollProps {
  loadMore: () => Promise<boolean>;
  hasMore: boolean;
  isLoading: boolean;
  loadingComponent?: React.ReactNode;
  endComponent?: React.ReactNode;
  threshold?: number;
  children: React.ReactNode;
}

export default function InfiniteScroll({
  loadMore,
  hasMore,
  isLoading,
  loadingComponent = (
    <div className="py-4 text-center">Loading more items...</div>
  ),
  endComponent = <div className="py-4 text-center">No more items to load</div>,
  threshold = 0.8,
  children,
}: InfiniteScrollProps) {
  const [isFetching, setIsFetching] = useState(false);
  const observerRef = useRef<IntersectionObserver | null>(null);
  const loadMoreRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (observerRef.current) {
      observerRef.current.disconnect();
    }

    observerRef.current = new IntersectionObserver(
      (entries) => {
        if (entries.length === 0) return;
        const first = entries[0];
        if (first.isIntersecting && hasMore && !isLoading && !isFetching) {
          handleLoadMore();
        }
      },
      { threshold },
    );

    const currentLoadMoreRef = loadMoreRef.current;
    if (currentLoadMoreRef) {
      observerRef.current.observe(currentLoadMoreRef);
    }

    return () => {
      if (observerRef.current) {
        observerRef.current.disconnect();
      }
    };
  }, [hasMore, isLoading, isFetching, threshold]);

  const handleLoadMore = async () => {
    if (!hasMore || isLoading || isFetching) return;

    setIsFetching(true);
    try {
      await loadMore();
    } catch (error) {
      console.error("Error loading more items:", error);
    } finally {
      setIsFetching(false);
    }
  };

  return (
    <div className="infinite-scroll-container">
      {children}

      <div ref={loadMoreRef} className="load-more-trigger">
        {isLoading || isFetching
          ? loadingComponent
          : hasMore
            ? null
            : endComponent}
      </div>
    </div>
  );
}
